/**
 * @file   movableRobotBlockCode.cpp
 * @author Bogdan <bogdan@gorelkin.me>, RAHMAN <motiur@ieee.org>, Mona <mona.rahbari@utbm.fr>
 * @date   Fri Nov 27 23:09:23 2020
 *
 * @brief
 * 
 *       In the course of this work, we encountered the following difficulties:
 * 
 * 
 * 1) The first solution that we implemented was to manually set the parameters for movement for each robot.
 *     It certainly worked, but this implementation is inconvenient for large configurations
 *
 * 2) New solution - use of broadcasting. This solution allowed to get the number of connected modules
 * 3) Having implemented an automatic algorithm for the movement of modular robots, we solved the problem of
 *     using large configurations and brought our model closer to real conditions
 *
 * 4) The resulting model matched the expected but there were problems with the orientation of the robots.
 *     In order for the robot to return to its original orientation, the robot needs to walk n * 6 = steps. 
 *     In our form, two robots passed (step mode 6) = 5. 
 *     When modeling this situation, no problems arise, but under real conditions, the magnetic fields of the connecting robots may have conflicts.
 *     Understanding the causes of this problem, we got to the core of the program and tried to solve the problem in the hexanodesMotionEngine.h file. 
 *     Unfortunately editing .h manually did not lead us to the desired result,
 *     and updates of these files from October 16 create errors when compiling a visible sim program.
 * 
**/


#include "movableRobotBlockCode.hpp"
#include "robots/hexanodes/hexanodesWorld.h"

//for delay
#include <chrono>
#include <thread>

#include <unistd.h>
#include "events/scheduler.h"
#include "events/events.h"
#include "utils/trace.h"
#include "robots/hexanodes/hexanodesMotionEvents.h"
#include "robots/hexanodes/hexanodesMotionEngine.h"

using namespace Hexanodes;

MovableRobotBlockCode::MovableRobotBlockCode(HexanodesBlock *host) : HexanodesBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

     addMessageEventFunc2(BROADCAST_MSG_ID,
                       std::bind(&MovableRobotBlockCode::myBroadcastFunc,this,
                       std::placeholders::_1, std::placeholders::_2));

    // Registers a callback (myAcknowledgeFunc) to the message of type K
    addMessageEventFunc2(ACKNOWLEDGE_MSG_ID,
                       std::bind(&MovableRobotBlockCode::myAcknowledgeFunc,this,
                       std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(NEXT_MSG_ID,
                       std::bind(&MovableRobotBlockCode::acknowledgeNextFunc,this,
                       std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(LEADER_FOUND_MSG_ID,
                       std::bind(&MovableRobotBlockCode::NewLeader,this,
                       std::placeholders::_1, std::placeholders::_2));

    scheduler = getScheduler();
    node = (HexanodesBlock*)hostBlock;
  }

// Function called by the module upon initialization
void MovableRobotBlockCode::startup() {
    
    //Each robot knows if a cell of the lattice is inside the target or not using the following code.
    //Here we check which robots will stay in place (target) .
    if (target && target->isInTarget(node->position)) {
        //following color based on tutorial 
        //https://etudiants-stgi.pu-pm.univ-fcomte.fr/tp_bpiranda/matiereProgrammable/vs2.html
            if (Hexanodes::getWorld()->maxBlockId > 14)
            {
                node->setColor(RED);
            }else
            {
                node->setColor(CYAN);
            }
            
            
            inPosition = true;
        }
    //from leader the broadcast message will be sent to all neighbors
    if (isLeader) {
        currentRound=1;
        distance=0;
        nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,1)",new MessageOf<pair<int,int>>(BROADCAST_MSG_ID,make_pair(distance+1,currentRound)),1000,100,0);
	}else {
        currentRound=0;
    }
}

//following function will broadcast the message. With helping of this function we can get
//the distance between each robots 
void MovableRobotBlockCode::myBroadcastFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    
    MessageOf<pair<int,int>>* msg = static_cast<MessageOf<pair<int,int>>*>(_msg.get());
    pair<int,int> msgData = *msg->getData();

    if (isLeader){
            isLeader = false;
            //cout <<  "Debugging!"<< "\n"<<endl;
        }

    if (parent==nullptr || msgData.first<distance || msgData.second > currentRound) {
        distance=msgData.first;
        NextFromLeader = node->blockId;

        if(inPosition){
            distanceOfNext = 0;
        }
        else{
            distanceOfNext = distance;
        }

        currentRound=msgData.second;
        parent=sender;
        string str="distance(";
        str+=to_string(distance+1)+","+to_string(currentRound)+")";
        nbWaitedAnswers=sendMessageToAllNeighbors(str.c_str(),new MessageOf<pair<int,int>>(BROADCAST_MSG_ID,make_pair(distance+1,currentRound)),1000,100,1,sender);
        
        if (nbWaitedAnswers==0) {
            sendMessage("acknowledgementToParent",new MessageOf<pair<int,int>>(NEXT_MSG_ID, make_pair(node->blockId,distance)),parent,1000,100);
        }

    } else {
        sendMessage("acknowledgementToSender",new MessageOf<pair<int,int>>(NEXT_MSG_ID, make_pair(NextFromLeader,distanceOfNext)),sender,1000,100);
     } 
}

//This function will return the back message to the parents
void MovableRobotBlockCode::myAcknowledgeFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    nbWaitedAnswers--;
    if (nbWaitedAnswers==0) {
        if (parent!=nullptr) {
            sendMessage("acknowledgementToParent",new Message(ACKNOWLEDGE_MSG_ID),parent,1000,100);
            
        } else {
            cout <<  "parrent null!"<< "\n"<<endl;
        }
    }
}




//this function will send the information about new leader
void MovableRobotBlockCode::acknowledgeNextFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    nbWaitedAnswers--;


    MessageOf<pair<int,int>>* msg = static_cast<MessageOf<pair<int,int>>*>(_msg.get());
    pair<int,int> msgData = *msg->getData();

    if (msgData.second > distanceOfNext){
        NextFromLeader =  msgData.first;
        distanceOfNext=msgData.second;
    }

    if(msgData.second == distanceOfNext && NextFromLeader < msgData.first){
      NextFromLeader = msgData.first;
      distanceOfNext = msgData.second;
    }

    if (nbWaitedAnswers==0) {

        if (parent!=nullptr) {
            sendMessage("acknowledgementToParent",new MessageOf<pair<int,int>>(NEXT_MSG_ID, make_pair(NextFromLeader,distanceOfNext)),parent,1000,100);
            
        } else {
              cout <<  "parrent null!"<< "\n"<<endl;
        }
        

        if(isLeader && nbWaitedAnswers == 0){
            currentRound++;
            nbWaitedAnswers=sendMessageToAllNeighbors("Broadcasting new leader!",new MessageOf<pair<int,int>>(LEADER_FOUND_MSG_ID,make_pair(NextFromLeader,currentRound)),1000,100,0);
            return;
        }
    }
}




//Following function is selecting the new leader 
void MovableRobotBlockCode::NewLeader(std::shared_ptr<Message>_msg,P2PNetworkInterface *sender){
    MessageOf<pair<int,int>>* msg = static_cast<MessageOf<pair<int,int>>*>(_msg.get());
    pair<int,int> msgData = *msg->getData();

    if ((unsigned)msgData.first != node->blockId && currentRound < msgData.second) {
        parent=sender;
        currentRound = msgData.second;
        string str="distance(";
        str+=to_string(distance+1)+","+to_string(currentRound)+")";
        nbWaitedAnswers=sendMessageToAllNeighbors(str.c_str(),new MessageOf<pair<int,int>>(LEADER_FOUND_MSG_ID,make_pair(msgData.first,msgData.second)),1000,100,1,sender);

        if (nbWaitedAnswers==0) {
            sendMessage("acknowledgementToParent",new Message(ACKNOWLEDGE_MSG_ID),parent,1000,100);
        }


    } else {
        sendMessage("acknowledgementToSender",new Message(ACKNOWLEDGE_MSG_ID),sender,1000,100);
    }



    if ((unsigned)msgData.first == node->blockId && currentRound < msgData.second){

        if(inPosition){
            return;
        }


        sendMessage("acknowledgementToParent",new Message(ACKNOWLEDGE_MSG_ID),parent,1000,100);
        isLeader = true;
        currentRound = msgData.second;
        distance=0;
        parent = nullptr;
        distanceOfNext=0;
        vector<HexanodesMotion*> tab = Hexanodes::getWorld()->getAllMotionsForModule(node);
        auto ci=tab.begin();

        while (ci!=tab.end() && ((*ci)->direction!=motionDirection::CW)) {
            ci++;
        }

        if (ci!=tab.end()) {
            Cell3DPosition destination = (*ci)->getFinalPos(node->position);
            auto orient = (*ci)->getFinalOrientation(node->orientationCode);
            scheduler->schedule(new HexanodesMotionStartEvent(scheduler->now()+100000, node,destination,orient));
        }
   
    }
}



//Function is moving your robots just try its amazing
void MovableRobotBlockCode::onMotionEnd() {
    
    //this function make a delay so which increasing value 
    // moving will be slowly and your algohritm more understandable
    //this_thread::sleep_for(chrono::milliseconds(50) );

    //following color based on tutorial 
    //to get red color on robots which is moved
    //https://etudiants-stgi.pu-pm.univ-fcomte.fr/tp_bpiranda/matiereProgrammable/vs2.html
    if (Hexanodes::getWorld()->maxBlockId > 14)
    {
        setColor(RED);
    }
    
    

    nMotions++;

    vector<HexanodesMotion*> tab = Hexanodes::getWorld()->getAllMotionsForModule(node);
    
    auto ci = tab.begin();
    Cell3DPosition destination;

    while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
      ci++;
    }

   
   
    if (ci != tab.end()) {
      destination = (*ci) -> getFinalPos(node->position);
    }

    if ((target && target->isInTarget(node->position) && node->getNbNeighbors()>2) || (target&&target->isInTarget(node->position)&& !target->isInTarget(destination))) {
            inPosition = true;

        }

    if (inPosition){

        distance=0;
        currentRound++;
        nbWaitedAnswers=sendMessageToAllNeighbors("distance(1,1)",new MessageOf<pair<int,int>>(BROADCAST_MSG_ID,make_pair(distance+1,currentRound)),1000,100,0);
    } else{
        vector<HexanodesMotion*> tab = Hexanodes::getWorld()->getAllMotionsForModule(node);
        auto ci=tab.begin();
        
        while (ci!=tab.end() && ((*ci)->direction!=motionDirection::CW)) {
            ci++;
        }

        if (ci!=tab.end()) {
            Cell3DPosition destination = (*ci)->getFinalPos(node->position);
            auto orient = (*ci)->getFinalOrientation(node->orientationCode);
            scheduler->schedule(new HexanodesMotionStartEvent(scheduler->now()+100000, node,destination,orient));

        }

    }
}

//Display anything on the intarface of VisibleSim
string MovableRobotBlockCode::onInterfaceDraw() {

    stringstream trace;
    trace << "Number of modules: "+ to_string(Hexanodes::getWorld()->maxBlockId)<< " and ";
    trace << "Number of motions: " + to_string(nMotions)<< "";
    return trace.str();
}



void MovableRobotBlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("leader");
    if (attr!=nullptr) {
        string str(attr);
        if (str=="true" || str=="1" || str=="yes") {
            isLeader=true;
            std::cout << node->blockId << " is Leader!" << std::endl; // complete with your code
        }
    }
}


void MovableRobotBlockCode::processLocalEvent(EventPtr pev) {
    std::shared_ptr<Message> message;
    stringstream info;

    // Do not remove line below
    HexanodesBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_ADD_NEIGHBOR: {
            // Do something when a neighbor is added to an interface of the module
            break;
        }

        case EVENT_REMOVE_NEIGHBOR: {
            // Do something when a neighbor is removed from an interface of the module
            break;
        }
    }
}

/// ADVANCED BLOCKCODE FUNCTIONS BELOW

void MovableRobotBlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << "\n" << "--- PRINT MODULE " << *node << "---" << "\n";
}

void MovableRobotBlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool MovableRobotBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    /* Reading the command line */
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {

            // Single character example: -b
            case 'b':   {
                console << "-b option provided" << "\n";
                return true;
            } break;

            // Composite argument example: --foo 13
            case '-': {
                string varg = string((*argv)[0] + 2); // argv[0] without "--"
                if (varg == string("foo")) { //
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch(std::logic_error&) {
                        stringstream err;
                        err << "foo must be an integer. Found foo = " << argv[1] << "\n";
                        throw CLIParsingError(err.str());
                    }

                    console << "--foo option provided with value: " << fooArg << "\n";
                } else return false;

                return true;
            }

            default: cerr << "Unrecognized command line argument: " << (*argv)[0] << "\n";
        }
    }

    return false;
}
