#include <glog/logging.h>
#include <iostream>
#include <fstream>
extern "C" {
#include <common/my_kbhit.h>
}
#include <h_bot/h_bot.h>


DEFINE_string(serialName, "/dev/ttyUSB0", "serial port name");

using namespace std;
int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  //PIRoBot::HBot * pHbot = new PIRoBot::HBot();
  PIRoBot::Bot * pBot = new PIRoBot::HBot();
  PIRoBot::HBot * pHbot = dynamic_cast<PIRoBot::HBot*>(pBot);
  bool ret = pBot->InitBot(FLAGS_serialName);
  if(!ret)
    return -1;
  pBot->StartBotThread();
  init_kbhit();
  double curLinearV = 0.0;
  double curAngularV = 0.0;
  double stepLinearV = 0.05;
  double stepAngularV = 0.05;
  double maxLinearV = 0.25;
  double maxAngularV = 1.5;
  //pBot->SetLinearAndAugularVelocity Unit is m/s and rad/s
  ofstream botFile;
  botFile.open("bot_data.txt", ios::out|ios::in);
  ofstream botVelFile;  // storage the velocity of wheel instead of encoder
  botVelFile.open("bot_vel_data.txt", ios::out|ios::in);
  double leftVel, rightVel;
  double lastLeftEncoder = 0, lastRightEncoder = 0;
  size_t lastTimestamp;
  double mechParam = 2*3.14*0.0325/(48*46.852);
  bool notFirst = false;
  double mySnakeRotRad[] = {0, 1.57, 3.14, 1.57};
  int mySnakeRotNum = 0;
  double mySnakeTranLongLen = 1.0;
  double mySnakeTranShortLen = 0.1;
  double squareDist = 1.0;

  while(1) {
    usleep(1000);
    if(kbhit()) {
      const int key = getchar();
      if(key == 'z') {
        cout<<"stop"<<endl;
        curLinearV = 0;
        curAngularV = 0;
        pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
      }
      else if(key == 'w') {
        curLinearV += stepLinearV;
        curLinearV = (curLinearV < maxLinearV) ? curLinearV : maxLinearV;
        curAngularV = 0;
        pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
      }
      else if(key == 's') {
        curLinearV -= stepLinearV;
        curLinearV = (curLinearV > -maxLinearV) ? curLinearV : -maxLinearV;
        curAngularV = 0;
        pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
      }
      else if(key == 'a') {
        cout<<"rotate left"<<endl;
        curAngularV += stepAngularV;
        curAngularV = (curAngularV < maxAngularV) ? curAngularV : maxAngularV;
        curLinearV = 0;
        pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
      }
      else if(key == 'd') {
        cout<<"rotate right"<<endl;
        curAngularV -= stepAngularV;
        curAngularV = (curAngularV > -maxAngularV) ? curAngularV : -maxAngularV;
        curLinearV = 0;
        pBot->SetLinearAndAugularVelocity(curLinearV, curAngularV);
      }
      else if(key == 'e') {
        cout<<"enable sensor state send"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->EnableSensorStateSend();
        notFirst = false;
      }
      else if(key == 'r') {
        cout<<"disable sensor state send"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->DisableSensorStateSend();
      }
      else if(key == 'c') {
        cout<<"stop bot"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->StopBot();
      }
#if defined(HBotProtocolVer201701) || defined(HBotProtocolVer201703)
      else if(key == 'j') {
        cout<<"move dist"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->SetMoveDist(0.4, 0);
      }
      else if(key == 'k') {
        cout<<"snake"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->SetMoveMode(MoveModeSnakeWalk);
      }
      else if(key == 'l') {
        cout<<"square"<<endl;
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        pHbot->SetMoveSquare(squareDist);
      }
      else if(key == 'o') {
        if(squareDist < 4) {
          squareDist += 0.2;
        }
        cout<<"square dist: "<<squareDist<<endl;
      }
      else if(key == 'i') {
        if(squareDist > 0.2) {
          squareDist -= 0.2;
        }
        cout<<"square dist: "<<squareDist<<endl;
      }
      else if(key == 'm') {
        pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
        CHECK(pHbot);
        if(mySnakeRotNum == 0 || mySnakeRotNum == 2) {
          pHbot->SetMoveDist(mySnakeTranLongLen, mySnakeRotRad[mySnakeRotNum]);
        }
        else if(mySnakeRotNum == 1 || mySnakeRotNum == 3) {
          pHbot->SetMoveDist(mySnakeTranShortLen, mySnakeRotRad[mySnakeRotNum]);
        }
        mySnakeRotNum ++;
        mySnakeRotNum %= 4;
      }
      else if(key == 'n') {
        if(mySnakeTranLongLen < 5) {
          mySnakeTranLongLen +=0.2;
        }
        cout<<"my snake translate long len: "<<mySnakeTranLongLen<<endl;
      }
      else if(key == 'b') {
        if(mySnakeTranLongLen > 0.2) {
          mySnakeTranLongLen -=0.2;
        }
        cout<<"my snake translate long len: "<<mySnakeTranLongLen<<endl;
      }
      else if(key == 'h') {
        if(mySnakeTranShortLen < 2) {
          mySnakeTranShortLen +=0.05;
        }
        cout<<"my snake translate long len: "<<mySnakeTranShortLen<<endl;
      }
      else if(key == 'g') {
        if(mySnakeTranShortLen >= 0.05) {
          mySnakeTranShortLen -=0.05;
        }
        cout<<"my snake translate long len: "<<mySnakeTranShortLen<<endl;
      }
#endif
      else if(key == 0x1b || key == 'q') {
        pBot->mutexExitThread.lock();
        pBot->isExitThread = true;
        pBot->mutexExitThread.unlock();

        break;
      }
    }
    pBot->mutexStateUpdate.lock();
    if(pBot->isNewState) {
      pBot->isNewState = false;
      //cout<<"cur pos "<<pBot->pos.x<<" "<<pBot->pos.y<<" "<<pBot->pos.th<<" "<<pBot->leftEncoderPos<<" "<<pBot->rightEncoderPos<<endl;
      pHbot = dynamic_cast<PIRoBot::HBot *>(pBot);
      CHECK(pHbot);
      size_t timestamp;
      //get13bitTimestamp(&timestamp);
      timestamp = pHbot->timestamp;
      //gettimeofday(&curTime, NULL);
      if(notFirst) {
        botFile<<timestamp<<" "<<pHbot->pos.x<<" "<<pHbot->pos.y<<" "<<pHbot->pos.th<<" "<<pHbot->leftEncoderPos<<" "<<pHbot->rightEncoderPos<<" "<<pHbot->gyrZ<<" "<<pHbot->headingFromImu<<endl;
        double lastTime, curTime;
        lastTime = lastTimestamp;
        curTime = timestamp;
        double deltaTime = (curTime - lastTime) / 1000.0;
        //double deltaTime = (curTime.tv_sec + static_cast<double>(curTime.tv_usec) / 1e6 - lastTime.tv_sec - static_cast<double>(lastTime.tv_usec) / 1e6);
        if(deltaTime < 0.010 || deltaTime > 0.030) {
          //cout<<"wrong dt:"<<deltaTime<<" "<<curTime.tv_sec + static_cast<double>(curTime.tv_usec) / 1e6<<" "<<lastTime.tv_sec + static_cast<double>(lastTime.tv_usec) / 1e6<<endl;
          cout<<"wrong dt:"<<deltaTime<<" "<<curTime<<" "<<lastTime<<endl;
        }
        leftVel = (pHbot->leftEncoderPos - lastLeftEncoder) * mechParam / deltaTime;
        rightVel = (pHbot->rightEncoderPos - lastRightEncoder) * mechParam / deltaTime;
        botVelFile<<timestamp<<" "<<pHbot->pos.x<<" "<<pHbot->pos.y<<" "<<pHbot->pos.th<<" "<<leftVel<<" "<<rightVel<<" "<<pHbot->gyrZ<<" "<<pHbot->headingFromImu<<endl;
      }
      else {
        notFirst = true;
      }
      lastTimestamp = timestamp;
      lastLeftEncoder = pHbot->leftEncoderPos;
      lastRightEncoder = pHbot->rightEncoderPos;
      //cout<<"cur sensor "<<
    }
    pBot->mutexStateUpdate.unlock();
  }
  botFile.close();
  finish_kbhit();
  pBot->EndBotThread();
  pBot->Finish();
}
