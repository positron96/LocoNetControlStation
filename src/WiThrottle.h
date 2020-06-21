/*
 * Based on https://github.com/positron96/withrottle
 */

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>

#include "CommandStation.h"
#include "debug.h"

/* Maximum WiFi clients that can be connected to WiThrottle */
#define MAX_CLIENTS 3

/* Network parameters */
#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED 2
#define TURNOUT_THROWN 4


String powerStatus;

#define LOG_WIFI 0

inline int invert(int value) {
    return value == 0 ? 1 : 0;
}

class WiThrottleServer {
public:

    WiThrottleServer(uint16_t port=44444) : port(port), server(port) {}

    void begin() {

        DEBUGS("Connected");

        server.begin();

        //MDNS.begin(hostString);
        MDNS.addService("withrottle","tcp", port);
        //MDNS.setInstanceName("DCC++ Network Interface");

    }


    void loop() {
        
        for (int iClient=0; iClient<MAX_CLIENTS; iClient++) {
            WiFiClient& cli = clients[iClient];
            if (!cli) {
                cli = server.available();
            }
            else {
                if (cli.status() == CLOSED) {
                    throttleStop(iClient);
                }
                else if (!alreadyConnected[iClient]) {
                    loadTurnouts();
                    throttleStart(iClient);
                }
            }
            if (cli.available()) {
                //bool changeSpeed[] = { };
                while(cli.available()>0) { 
                    String clientData = cli.readStringUntil('\n'); 
                    if (LOG_WIFI) DEBUGS("WF>> "+clientData);
                    if (clientData.startsWith("*+")) {
                        heartbeatEnable[iClient] = true;
                    }
                    else if (clientData.startsWith("PPA")) {
                        turnPower(clientData.charAt(3));
                        //notifyPowerChange();
                    }
                    else if (clientData.startsWith("PTA")) {
                        char aStatus = clientData.charAt(3);
                        int aAddr;
                        bool named;
                        if(clientData.substring(4,6)==TURNOUT_PREF) {
                            // named turnout
                            aAddr = clientData.substring(6).toInt();
                            named = true;
                        } else {
                            aAddr = clientData.substring(4).toInt();
                            named = false;
                        }
                        accessoryToggle(aAddr, aStatus, named);
                    }
                    else if (clientData.startsWith("N") 
                        || clientData.startsWith("*")) {
                        wifiPrintln(iClient, "*" + String(heartbeatTimeout));
                    }
                    else if (clientData.startsWith("MT") 
                        || clientData.startsWith("MS") 
                        || clientData.startsWith("M0") 
                        || clientData.startsWith("M1")) {
                        char th = clientData.charAt(1);
                        int iThrottle;
                        if (th == 'T' || th == '0')
                            iThrottle = 0+iClient*2;
                        else
                            iThrottle = 1+iClient*2;
                        char action = clientData.charAt(2);
                        String actionData = clientData.substring(3);
                        int delimiter = actionData.indexOf(";");
                        String actionKey = actionData.substring(0, delimiter-1);
                        String actionVal = actionData.substring(delimiter+2);
                        if (action == '+') {
                            locoAdd(th, actionKey, iThrottle, iClient);
                        }
                        else if (action == '-') {
                            locoRelease(th, actionKey, iThrottle, iClient);
                        }
                        else if (action == 'A') {
                            locoAction(th, actionKey, actionVal, iThrottle, iClient);
                        }
                        heartbeat[iThrottle] = millis();
                    }
                }
                if (heartbeatEnable[iClient]) {
                    checkHeartbeat(iClient);
                }
            }
        }
    }

private:

    const uint16_t port;

    /* The interval of check connections between ESP & WiThrottle app */
    const unsigned int heartbeatTimeout = 10;
    bool heartbeatEnable[MAX_CLIENTS];
    unsigned long heartbeat[MAX_CLIENTS*2];

    String locoAddesses[MAX_CLIENTS*2] = {""};
    int locoStates[MAX_CLIENTS*2][31];

    boolean alreadyConnected[MAX_CLIENTS];

    /* Define WiThrottle Server */
    WiFiServer server;
    WiFiClient clients[MAX_CLIENTS];


    void notifyPowerStatus() {
        for (int p=0; p<MAX_CLIENTS; p++) {
            if (alreadyConnected[p]) {
                clients[p].println("PPA"+powerStatus);
            }
        }
    }

    void turnPower(char v) {
        sendDCCppCmd(String(v));
    }


    void loadTurnouts() {
        sendDCCppCmd("T");
        waitForDCCpp();
        int t = 0;
        while(Serial.available()>0) {
            char data[maxCommandLength];
            sprintf(data, "%s", readResponse().c_str() );
            if (strlen(data)==0) break;
            int addr, sub, stat, id;
            int ret = sscanf(data, "%*c %d %d %d %d", &id, &addr, &sub, &stat );
            turnoutData[t] = { ((addr-1)*4+1) + (sub&0x3) , id, stat==0 ? 2 : 4};
            t++;
        }
    }

    void wifiPrintln(int iClient, String v) {
        clients[iClient].println(v);
        if (LOG_WIFI) DEBUGS("WF<< "+v);
    }
    void wifiPrint(int iClient, String v) {
        clients[iClient].print(v);
        if (LOG_WIFI) DEBUGS("WF<< "+v);
    }

    void throttleStart(int iClient) {
        clients[iClient].flush();
        clients[iClient].setTimeout(500);
        DEBUGS("New client");

        wifiPrintln(iClient, "VN2.0");
        wifiPrintln(iClient, "RL0");
        wifiPrintln(iClient, "PPA"+powerStatus);
        wifiPrintln(iClient, "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{"+String(TURNOUT_CLOSED)+"]\\[Thrown}|{"+String(TURNOUT_THROWN) );
        wifiPrint(iClient, "PTL");
        for (int t = 0 ; turnoutData[t].address != 0; t++) {
            wifiPrint(iClient, String("]\\[")+TURNOUT_PREF+turnoutData[t].address+"}|{"+turnoutData[t].id+"}|{"+turnoutData[t].tStatus);
        }
        wifiPrintln(iClient, "");
        wifiPrintln(iClient, "*"+String(heartbeatTimeout));
        alreadyConnected[iClient] = true;
    }

    void throttleStop(int iClient) {
        clients[iClient].stop();
        DEBUGS("Client lost");
        alreadyConnected[iClient] = false;
        heartbeatEnable[iClient] = false;
        locoStates[0+iClient*2][29] = 0;	 heartbeat[0+iClient*2] = 0;
        locoStates[1+iClient*2][29] = 0;	 heartbeat[1+iClient*2] = 0;
    }

    void locoAdd(char th, String locoAddr, int iThrottle, int iClient) {
        locoAddesses[iThrottle] = locoAddr;
        wifiPrintln(iClient, String("M")+th+"+"+locoAddr+"<;>");
        for (int fKey=0; fKey<29; fKey++) {
            locoStates[iThrottle][fKey] =0;
            wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>F0"+String(fKey));
        }
        locoStates[iThrottle][29] =0;
        locoStates[iThrottle][30] =1;
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>V0");
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>R1");
        wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>s1");
        DEBUGS("loco add thr="+String(iThrottle)+"; addr"+String(locoAddr) );
    }

    void locoRelease(char th, String locoAddr, int iThrottle, int iClient) {
        String locoAddress = locoAddesses[iThrottle].substring(1);
        heartbeat[iThrottle] =0;
        locoAddesses[iThrottle] = "";
        wifiPrintln(iClient, String("M")+th+"-"+locoAddr+"<;>");
        DEBUGS("loco release thr="+String(iThrottle)+"; addr"+String(locoAddr) );
        // stop now
        sendDCCppCmd(String("t ")+String(iThrottle+1)+" "+locoAddress+" 0 "+String(locoStates[iThrottle][30]));
        String response = loadResponse();
    }

    void locoAction(char th, String locoAddr, String actionVal, int iThrottle, int i) {
        String response;
        if (locoAddr == "*") {
            locoAddr = locoAddesses[iThrottle];
        }
        String dccLocoAddr = locoAddr.substring(1);
        DEBUGS("loco action thr="+String(iThrottle)+"; action="+actionVal+"; DCC"+String(dccLocoAddr) );
        int *locoState = locoStates[iThrottle];
        if (actionVal.startsWith("F1")) {
            int fKey = actionVal.substring(2).toInt();
            locoState[fKey] = invert(locoState[fKey]);
            wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "F"+String(locoState[fKey])+String(fKey));
            byte func;
            switch(fKey) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    func = 128
                        + locoState[1]*1
                        + locoState[2]*2
                        + locoState[3]*4
                        + locoState[4]*8
                        + locoState[0]*16;
                    sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
                break;
                case 5:
                case 6:
                case 7:
                case 8:
                    func = 176
                        + locoState[5]*1
                        + locoState[6]*2
                        + locoState[7]*4
                        + locoState[8]*8;
                    sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
                break;
                case 9:
                case 10:
                case 11:
                case 12:
                    func = 160
                    + locoState[9]*1
                    + locoState[10]*2
                    + locoState[11]*4
                    + locoState[12]*8;
                    sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
                break;
                case 13:
                case 14:
                case 15:
                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
                    func = locoState[13]*1 
                        + locoState[14]*2
                        + locoState[15]*4
                        + locoState[16]*8
                        + locoState[17]*16
                        + locoState[18]*32
                        + locoState[19]*64
                        + locoState[20]*128;
                    sendDCCppCmd("f "+dccLocoAddr+" "+String(222)+" "+String(func));
                break;
                case 21:
                case 22:
                case 23:
                case 24:
                case 25:
                case 26:
                case 27:
                case 28:
                    func = locoState[21]*1
                        + locoState[22]*2
                        + locoState[23]*4
                        + locoState[24]*8
                        + locoState[25]*16
                        + locoState[26]*32
                        + locoState[27]*64
                        + locoState[28]*128;
                    sendDCCppCmd("f "+dccLocoAddr+" "+String(223)+" "+String(func));
                break;
            }
        }
        else if (actionVal.startsWith("qV")) {
            //DEBUGS("query speed for loco "+String(dccLocoAddr) );
            wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "V"+String(locoState[29]));							
        }
        else if (actionVal.startsWith("V")) {
            //DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
            locoState[29] = actionVal.substring(1).toInt();
            sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));
            response = loadResponse();
        }
        else if (actionVal.startsWith("qR")) {
            //DEBUGS("query dir for loco "+String(dccLocoAddr) );
            wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "R"+String(locoState[30]));							
        }
        else if (actionVal.startsWith("R")) {
            //DEBUGS("Sending dir to addr "+String(dccLocoAddr) );
            locoState[30] = actionVal.substring(1).toInt();
            sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" "+String(locoState[29])+"	"+String(locoState[30]));
            response = loadResponse();
        }
        else if (actionVal.startsWith("X")) {
            locoState[29] = 0;
            sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
            response = loadResponse();
        }
        else if (actionVal.startsWith("I")) {
            locoState[29] = 0;
            sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
            response = loadResponse();
        }
        else if (actionVal.startsWith("Q")) {
            locoState[29] = 0;
            sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
            response = loadResponse();
        }
    }

    void checkHeartbeat(int iClient) {
        for (int iThrottle=0; iThrottle<2; iThrottle++) {
            int ii = iThrottle + iClient*2;
            if (heartbeat[ii] > 0 && heartbeat[ii] + heartbeatTimeout * 1000 < millis()) {
                // stop loco
                //TODO: is it sent to track?
                locoStates[ii][29] = 0;
                heartbeat[ii] = 0;
                wifiPrintln(iClient, (iThrottle==0?"MTA":"MSA")+locoAddesses[ii]+"<;>" + "V0");
            }
        }
    }

    void accessoryToggle(int aAddr, char aStatus, bool namedTurnout) {
        TurnoutState newStat = TurnoutState::TOGGLE;
        switch(aStatus) {
            case 'T': newStat=TurnoutState::THROWN; break;
            case 'C': newStat=TurnoutState::CLOSED; break;
        }

        DEBUGS(String("turnout action, addr=")+aAddr+"; named:"+namedTurnout );

        newStat = CS.turnoutAction(aAddr, namedTurnout, newStat);

        int wStat = 3; // unknown
        switch(newStat) {
            case TurnoutState::THROWN: wStat = TURNOUT_THROWN; break;
            case TurnoutState::CLOSED: wStat = TURNOUT_CLOSED; break;
        }

        for (int i=0; i<MAX_CLIENTS; i++) {
            if(clients[i])
            wifiPrintln(i, String("PTA")+wStat+(namedTurnout?TURNOUT_PREF:"")+aAddr);
        }

    }
};