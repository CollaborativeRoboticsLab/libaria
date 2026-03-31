#include <Aria/Aria.h>
#include <ArClientHandlerRobotUpdate.h>
#include <ArClientRatioDrive.h>
#include <ArNetworking.h>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
    const char *kDefaultHost = "172.19.21.203";
    const char *kDefaultPort = "7272";
    const char *kDefaultUser = "steve";

    bool hasArgument(int argc, char **argv, const char *argument)
    {
        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == argument)
            {
                return true;
            }
        }
        return false;
    }

    void applyRos1Defaults(int argc, char **argv, ArArgumentParser &parser)
    {
        if (!hasArgument(argc, argv, "-host"))
        {
            parser.addDefaultArgument("-host");
            parser.addDefaultArgument(kDefaultHost);
        }
        if (!hasArgument(argc, argv, "-p"))
        {
            parser.addDefaultArgument("-p");
            parser.addDefaultArgument(kDefaultPort);
        }
        if (!hasArgument(argc, argv, "-u"))
        {
            parser.addDefaultArgument("-u");
            parser.addDefaultArgument(kDefaultUser);
        }
        if (!hasArgument(argc, argv, "-pw") && !hasArgument(argc, argv, "-np"))
        {
            parser.addDefaultArgument("-np");
        }
    }

    double parseDouble(const std::string &text, const char *name)
    {
        char *end = NULL;
        const double value = std::strtod(text.c_str(), &end);
        if (end == text.c_str() || (end != NULL && *end != '\0'))
        {
            throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
        }
        return value;
    }

    int parseInt(const std::string &text, const char *name)
    {
        char *end = NULL;
        const long value = std::strtol(text.c_str(), &end, 10);
        if (end == text.c_str() || (end != NULL && *end != '\0'))
        {
            throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
        }
        return static_cast<int>(value);
    }

    void printHelp()
    {
        std::cout
            << "Commands:\n"
            << "  help\n"
            << "  status\n"
            << "  watch [count] [interval_ms]\n"
            << "  stop\n"
            << "  safe\n"
            << "  unsafe\n"
            << "  ratio <trans_pct> <rot_pct> [duration_ms] [throttle_pct] [lat_pct]\n"
            << "  cmdvel <linear> <angular> [duration_ms] [throttle_pct] [lat]\n"
            << "  goto <x_m> <y_m> <theta_deg>\n"
            << "  dock\n"
            << "  quit\n"
            << "\n"
            << "Connection defaults match the ROS 1 node: -host " << kDefaultHost
            << " -p " << kDefaultPort
            << " -u " << kDefaultUser << " -np\n";
    }

    class DockInfoWatcher
    {
    public:
        DockInfoWatcher() : myDockState(-1),
                            myForcedDock(-1),
                            mySecondsToShutdown(-1),
                            myHaveDockInfo(false),
                            myDockInfoChangedCB(this, &DockInfoWatcher::dockInfoChanged)
        {
        }

        ArFunctor1<ArNetPacket *> *callback()
        {
            return &myDockInfoChangedCB;
        }

        const char *stateString() const
        {
            switch (myDockState)
            {
            case 0:
                return "Undocked";
            case 1:
                return "Docking";
            case 2:
                return "Docked";
            case 3:
                return "Undocking";
            default:
                return "Unknown";
            }
        }

        const char *forcedDockString() const
        {
            switch (myForcedDock)
            {
            case 0:
                return "false";
            case 1:
                return "true";
            default:
                return "unknown";
            }
        }

        bool haveDockInfo() const
        {
            return myHaveDockInfo;
        }

        int secondsToShutdown() const
        {
            return mySecondsToShutdown;
        }

    private:
        void dockInfoChanged(ArNetPacket *packet)
        {
            myDockState = packet->bufToUByte();
            myForcedDock = packet->bufToUByte();
            mySecondsToShutdown = packet->bufToUByte2();
            myHaveDockInfo = true;
        }

        int myDockState;
        int myForcedDock;
        int mySecondsToShutdown;
        bool myHaveDockInfo;
        ArFunctor1C<DockInfoWatcher, ArNetPacket *> myDockInfoChangedCB;
    };

    void printStatus(ArClientHandlerRobotUpdate &updates, const DockInfoWatcher &dockInfo)
    {
        updates.lock();
        std::cout << std::fixed << std::setprecision(2)
                  << "Mode: " << updates.getMode() << '\n'
                  << "Status: " << updates.getStatus() << '\n'
                  << "Pose: x=" << updates.getX() / 1000.0
                  << " m y=" << updates.getY() / 1000.0
                  << " m th=" << updates.getTh() << " deg\n"
                  << "Velocity: trans=" << updates.getVel() / 1000.0
                  << " m/s lat=" << updates.getLatVel() / 1000.0
                  << " m/s rot=" << updates.getRotVel() << " deg/s\n"
                  << "Battery: " << updates.getVoltage() << " V\n";
        updates.unlock();

        if (dockInfo.haveDockInfo())
        {
            std::cout << "Dock: state=" << dockInfo.stateString()
                      << " forced=" << dockInfo.forcedDockString();
            if (dockInfo.secondsToShutdown() == 0)
            {
                std::cout << " shutdown=never\n";
            }
            else
            {
                std::cout << " shutdown=" << dockInfo.secondsToShutdown() << " s\n";
            }
        }
    }

    bool sendGotoPose(ArClientBase &client, double xMeters, double yMeters, double thetaDegrees)
    {
        if (!client.dataExists("gotoPose"))
        {
            std::cout << "Server does not advertise gotoPose.\n";
            return false;
        }

        ArNetPacket packet;
        packet.byte4ToBuf(static_cast<int>(std::lround(xMeters * 1000.0)));
        packet.byte4ToBuf(static_cast<int>(std::lround(yMeters * 1000.0)));
        packet.byte4ToBuf(static_cast<int>(std::lround(thetaDegrees)));
        client.requestOnce("gotoPose", &packet);
        return true;
    }

    bool sendDock(ArClientBase &client)
    {
        if (!client.dataExists("dock"))
        {
            std::cout << "Server does not advertise dock.\n";
            return false;
        }
        client.requestOnce("dock");
        return true;
    }

    void sendCmdVel(ArClientBase &client, double linear, double angular, double throttle, double lateral)
    {
        if (!client.dataExists("ratioDrive"))
        {
            throw std::runtime_error("Server does not advertise ratioDrive.");
        }

        ArNetPacket packet;
        packet.doubleToBuf(linear);
        packet.doubleToBuf(angular);
        packet.doubleToBuf(throttle);
        packet.doubleToBuf(lateral);
        client.requestOnce("ratioDrive", &packet);
    }
}

int main(int argc, char **argv)
{
    Aria::init();
    ArLog::init(ArLog::StdOut, ArLog::Normal);

    ArClientBase client;
    client.enforceProtocolVersion("5MTX");

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    applyRos1Defaults(argc, argv, parser);

    ArClientSimpleConnector clientConnector(&parser);
    if (!clientConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        printHelp();
        clientConnector.logOptions();
        Aria::exit(1);
        return 1;
    }

    if (!clientConnector.connectClient(&client))
    {
        if (client.wasRejected())
        {
            std::cerr << "Server '" << client.getHost() << "' rejected the connection.\n";
        }
        else
        {
            std::cerr << "Could not connect to server '" << client.getHost() << "'.\n";
        }
        Aria::exit(1);
        return 1;
    }

    client.setRobotName(client.getHost());
    client.runAsync();

    ArClientHandlerRobotUpdate updates(&client);
    updates.requestUpdates();

    DockInfoWatcher dockInfo;
    if (client.dataExists("dockInfoChanged"))
    {
        client.addHandler("dockInfoChanged", dockInfo.callback());
        client.requestOnce("dockInfoChanged");
        client.request("dockInfoChanged", -1);
    }

    ArClientRatioDrive ratioDrive(&client);
    ratioDrive.setThrottle(100.0);

    ArUtil::sleep(500);

    std::cout << "Connected to " << client.getHost() << ". Type 'help' for commands.\n";
    printStatus(updates, dockInfo);

    std::string line;
    while (client.getRunningWithLock())
    {
        std::cout << "omron> " << std::flush;
        if (!std::getline(std::cin, line))
        {
            break;
        }

        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command.empty())
        {
            continue;
        }

        try
        {
            if (command == "help")
            {
                printHelp();
            }
            else if (command == "status")
            {
                printStatus(updates, dockInfo);
            }
            else if (command == "watch")
            {
                int count = 10;
                int intervalMs = 1000;
                std::string arg;
                if (input >> arg)
                {
                    count = parseInt(arg, "count");
                }
                if (input >> arg)
                {
                    intervalMs = parseInt(arg, "interval_ms");
                }
                for (int i = 0; i < count && client.getRunningWithLock(); ++i)
                {
                    printStatus(updates, dockInfo);
                    ArUtil::sleep(intervalMs);
                }
            }
            else if (command == "stop")
            {
                ratioDrive.stop();
                std::cout << "Stop requested.\n";
            }
            else if (command == "safe")
            {
                ratioDrive.safeDrive();
                std::cout << "Safe drive requested.\n";
            }
            else if (command == "unsafe")
            {
                ratioDrive.unsafeDrive();
                std::cout << "Unsafe drive requested.\n";
            }
            else if (command == "ratio")
            {
                std::string transArg;
                std::string rotArg;
                if (!(input >> transArg >> rotArg))
                {
                    throw std::runtime_error("Usage: ratio <trans_pct> <rot_pct> [duration_ms] [throttle_pct] [lat_pct]");
                }

                const double trans = parseDouble(transArg, "trans_pct");
                const double rot = parseDouble(rotArg, "rot_pct");
                int durationMs = 0;
                double throttle = 100.0;
                double lateral = 0.0;
                std::string arg;

                if (input >> arg)
                {
                    durationMs = parseInt(arg, "duration_ms");
                }
                if (input >> arg)
                {
                    throttle = parseDouble(arg, "throttle_pct");
                }
                if (input >> arg)
                {
                    lateral = parseDouble(arg, "lat_pct");
                }

                ratioDrive.setThrottle(throttle);
                ratioDrive.setLatVelRatio(lateral);
                ratioDrive.setRotVelRatio(rot);
                ratioDrive.setTransVelRatio(trans);
                std::cout << "ratioDrive requested: trans=" << trans << " rot=" << rot
                          << " throttle=" << throttle << " lat=" << lateral << "\n";
                if (durationMs > 0)
                {
                    ArUtil::sleep(durationMs);
                    ratioDrive.stop();
                    std::cout << "Timed ratioDrive completed and stop requested.\n";
                }
            }
            else if (command == "cmdvel")
            {
                std::string linearArg;
                std::string angularArg;
                if (!(input >> linearArg >> angularArg))
                {
                    throw std::runtime_error("Usage: cmdvel <linear> <angular> [duration_ms] [throttle_pct] [lat]");
                }

                const double linear = parseDouble(linearArg, "linear");
                const double angular = parseDouble(angularArg, "angular");
                int durationMs = 0;
                double throttle = 100.0;
                double lateral = 0.0;
                std::string arg;

                if (input >> arg)
                {
                    durationMs = parseInt(arg, "duration_ms");
                }
                if (input >> arg)
                {
                    throttle = parseDouble(arg, "throttle_pct");
                }
                if (input >> arg)
                {
                    lateral = parseDouble(arg, "lat");
                }

                sendCmdVel(client, linear, angular, throttle, lateral);
                std::cout << "ratioDrive packet sent: linear=" << linear << " angular=" << angular
                          << " throttle=" << throttle << " lat=" << lateral << "\n";
                if (durationMs > 0)
                {
                    ArUtil::sleep(durationMs);
                    ratioDrive.stop();
                    std::cout << "Timed cmdvel completed and stop requested.\n";
                }
            }
            else if (command == "goto")
            {
                std::string xArg;
                std::string yArg;
                std::string thetaArg;
                if (!(input >> xArg >> yArg >> thetaArg))
                {
                    throw std::runtime_error("Usage: goto <x_m> <y_m> <theta_deg>");
                }

                const double x = parseDouble(xArg, "x_m");
                const double y = parseDouble(yArg, "y_m");
                const double theta = parseDouble(thetaArg, "theta_deg");
                if (sendGotoPose(client, x, y, theta))
                {
                    std::cout << "gotoPose requested: x=" << x << " m y=" << y << " m theta=" << theta << " deg\n";
                }
            }
            else if (command == "dock")
            {
                if (sendDock(client))
                {
                    std::cout << "Dock requested.\n";
                }
            }
            else if (command == "quit" || command == "exit")
            {
                break;
            }
            else
            {
                std::cout << "Unknown command: " << command << "\n";
                printHelp();
            }
        }
        catch (const std::exception &error)
        {
            std::cerr << error.what() << '\n';
        }
    }

    ratioDrive.stop();
    client.disconnect();
    Aria::exit(0);
    return 0;
}