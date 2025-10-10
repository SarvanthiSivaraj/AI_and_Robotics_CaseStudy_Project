#include "Walk.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR","ShoulderL","ArmUpperR","ArmUpperL","ArmLowerR","ArmLowerL",
  "PelvYR","PelvYL","PelvR","PelvL","LegUpperR","LegUpperL","LegLowerR",
  "LegLowerL","AnkleR","AnkleL","FootR","FootL","Neck","Head"
};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);

  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  getGyro("Gyro")->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
}

Walk::~Walk() {}

void Walk::myStep() {
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double start = getTime();
  while (getTime() < start + ms / 1000.0)
    myStep();
}

void Walk::run() {
  cout << "======= ROBOTIS OP2 AUTO WALK =======" << endl;
  cout << "Arrow keys: ↑ ↓ ← → | Space = stop | E = evolution" << endl;
  cout << "=====================================" << endl;

  myStep();
  mMotionManager->playPage(9);  // init pose
  wait(200);
  mGaitManager->start();

  double X = 1.0;
  double A = 0.0;
  double speed_step = 0.2;
  double turn_step = 0.2;

  double fitnessTime = 0;
  double fitnessStart = getTime();

  while (true) {
    // Check for falls
    checkIfFallen(true);

    // Get keyboard input
    int key = mKeyboard->getKey();
    switch (key) {
      case Keyboard::UP:
        X += speed_step;
        if (X > 2.0) X = 2.0;
        break;
      case Keyboard::DOWN:
        X -= speed_step;
        if (X < -1.0) X = -1.0;
        break;
      case Keyboard::LEFT:
        A += turn_step;
        if (A > 1.0) A = 1.0;
        break;
      case Keyboard::RIGHT:
        A -= turn_step;
        if (A < -1.0) A = -1.0;
        break;
      case ' ':
        X = 0.0;
        A = 0.0;
        break;
      case 'E':
      case 'e': {
        cout << "Starting evolutionary optimization..." << endl;

        struct Genotype { double x, a, fitness; };

        class Evolver {
        public:
          Evolver(Walk *w, int pop, int gen, int evalMs)
            : walk(w), populationSize(pop), generations(gen), evalTimeMs(evalMs) {
            rng.seed((unsigned)chrono::system_clock::now().time_since_epoch().count());
          }

          void run() {
            initPopulation();
            for (int g = 0; g < generations; ++g) {
              evaluateAll();
              vector<Genotype> next;
              keepBest(next);
              while ((int)next.size() < populationSize) {
                Genotype p1 = tournamentSelect();
                Genotype p2 = tournamentSelect();
                Genotype child = crossover(p1,p2);
                mutate(child);
                next.push_back(child);
              }
              population = next;
              cout << "Gen " << g << " best fitness " << bestEver.fitness
                   << " | X=" << bestEver.x << " | A=" << bestEver.a << endl;
            }
            cout << "Evolution done. Best fitness=" << bestEver.fitness
                 << " | X=" << bestEver.x << " | A=" << bestEver.a << endl;
            walk->mGaitManager->setXAmplitude(bestEver.x);
            walk->mGaitManager->setAAmplitude(bestEver.a);
          }

        private:
          Walk *walk;
          int populationSize;
          int generations;
          int evalTimeMs;
          vector<Genotype> population;
          Genotype bestEver;
          mt19937 rng;

          void initPopulation() {
            uniform_real_distribution<double> dx(0.0, 2.0);
            uniform_real_distribution<double> da(-1.0, 1.0);
            population.clear();
            for (int i=0;i<populationSize;i++){
              Genotype g = { dx(rng), da(rng), 0.0 };
              population.push_back(g);
            }
            bestEver = population[0];
          }

          void evaluateAll() {
            for(auto &ind : population){
              ind.fitness = evaluate(ind.x,ind.a);
              if(ind.fitness > bestEver.fitness) bestEver = ind;
            }
          }

          double evaluate(double x, double a){
            walk->mMotionManager->playPage(9);
            walk->mGaitManager->start();
            walk->mGaitManager->setXAmplitude(x);
            walk->mGaitManager->setAAmplitude(a);
            double start = walk->getTime();
            double now = start;
            while((now-start)*1000.0 < evalTimeMs){
              if(walk->checkIfFallen(false)) break;
              walk->myStep();
              now = walk->getTime();
            }
            double survived = (now-start)*1000.0;
            walk->mGaitManager->stop();
            walk->mMotionManager->playPage(9);
            return survived;
          }

          Genotype tournamentSelect(){
            uniform_int_distribution<int> di(0,populationSize-1);
            Genotype best = population[di(rng)];
            for(int i=0;i<2;i++){
              Genotype cand = population[di(rng)];
              if(cand.fitness > best.fitness) best = cand;
            }
            return best;
          }

          Genotype crossover(const Genotype &p1,const Genotype &p2){
            return Genotype{(p1.x+p2.x)/2.0,(p1.a+p2.a)/2.0,0.0};
          }

          void mutate(Genotype &g){
            normal_distribution<double> nx(0.0,0.1);
            normal_distribution<double> na(0.0,0.05);
            g.x += nx(rng); if(g.x<0) g.x=0;if(g.x>2) g.x=2;
            g.a += na(rng); if(g.a<-1) g.a=-1;if(g.a>1) g.a=1;
          }

          void keepBest(vector<Genotype> &next){
            sort(population.begin(),population.end(),
                 [](const Genotype &l,const Genotype &r){return l.fitness>r.fitness;});
            int keep = max(1,populationSize/5);
            for(int i=0;i<keep;i++) next.push_back(population[i]);
          }
        };

        Evolver ev(this, 10, 15, 4000);
        ev.run();
        cout << "Evolution applied." << endl;

      } break;
      default:
        break;
    }

    mGaitManager->setXAmplitude(X);
    mGaitManager->setAAmplitude(A);
    mGaitManager->step(mTimeStep);
    myStep();

    // Display real-time fitness every second
    if(getTime()-fitnessStart > 1.0){
      double currentFitness = getTime() - fitnessStart;
      cout << "Current X=" << X << " A=" << A << " | Fitness=" << currentFitness << "s" << endl;
      fitnessStart = getTime();
    }
  }
}

bool Walk::checkIfFallen(bool recover){
  static int fup=0,fdown=0;
  const double *acc = mAccelerometer->getValues();
  if(acc[1]<512-80) fup++; else fup=0;
  if(acc[1]>512+80) fdown++; else fdown=0;

  if(fup>100){
    if(recover){
      cout << "Robot fell forward. Getting up..." << endl;
      mMotionManager->playPage(10);
      mMotionManager->playPage(9);
      mGaitManager->start();
      fup=0;
      return false;
    } else { fup=0; return true; }
  } else if(fdown>100){
    if(recover){
      cout << "Robot fell backward. Getting up..." << endl;
      mMotionManager->playPage(11);
      mMotionManager->playPage(9);
      mGaitManager->start();
      fdown=0;
      return false;
    } else { fdown=0; return true; }
  }
  return false;
}