/*
 * File:   global.h
 * Author: pehladik
 *
 * Created on 21 avril 2011, 12:14
 */

#include "global.h"

RT_TASK tServeur;
RT_TASK tconnect;
RT_TASK tmove;
RT_TASK tenvoyer;
RT_TASK tbattery;
RT_TASK tcamera;

RT_MUTEX mutexEtat;
RT_MUTEX mutexMove;
RT_MUTEX mutexRobot;
RT_MUTEX mutexArena;
RT_MUTEX mutexPosition;

RT_SEM semConnecterRobot;
RT_SEM semGetImage;

RT_QUEUE queueMsgGUI;

int etatCommMoniteur = 1;
int etatCommRobot = 1;
DRobot *robot;
DMovement *move;
DServer *serveur;
DCamera *camera;
DMission *mission;
DBattery *battery;


int MSG_QUEUE_SIZE = 10;

int PRIORITY_TSERVEUR = 30;
int PRIORITY_TCONNECT = 20;
int PRIORITY_TMOVE = 15;
int PRIORITY_TENVOYER = 25;
int PRIORITY_TBATTERY = 10;
int PRIORITY_TCAMERA = 5;
