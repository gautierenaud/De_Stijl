/* 
 * File:   global.h
 * Author: pehladik
 *
 * Created on 12 janvier 2012, 10:11
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#include "includes.h"

/* @descripteurs des tâches */
extern RT_TASK tServeur;
extern RT_TASK tconnect;
extern RT_TASK tmove;
extern RT_TASK tenvoyer;
extern RT_TASK tbattery;
extern RT_TASK tcamera;
extern RT_TASK tverify;
extern RT_TASK tarena;
extern RT_TASK tcomputepos;

/* @descripteurs des mutex */
extern RT_MUTEX mutexEtat;
extern RT_MUTEX mutexMove;
extern RT_MUTEX mutexRobot;
extern RT_MUTEX mutexArena;
extern RT_MUTEX mutexPosition;
extern RT_MUTEX mutexImage;
extern RT_MUTEX mutexMission;

/* @descripteurs des sempahore */
extern RT_SEM semCommunicate;
extern RT_SEM semConnect;

extern RT_SEM semConnecterRobot;
extern RT_SEM semGetImage;
extern RT_SEM semDetectArena;
extern RT_SEM semComputePosition;
extern RT_SEM semwatchDog;

/* @descripteurs des files de messages */
extern RT_QUEUE queueMsgGUI;

/* @variables partagées */
extern int etatCommMoniteur;
extern int etatCommRobot;
extern DServer *serveur;
extern DRobot *robot;
extern DMovement *move;
extern DCamera *camera;
extern DMission *mission;
extern DBattery *battery;
extern DArena * arena;
extern DImage * image;
extern DPosition * position;
extern int compteur_dc;

/* @constantes */
extern int MSG_QUEUE_SIZE;
extern int PRIORITY_TSERVEUR;
extern int PRIORITY_TCONNECT;
extern int PRIORITY_TMOVE;
extern int PRIORITY_TENVOYER;
extern int PRIORITY_TBATTERY;
extern int PRIORITY_TCAMERA;
extern int PRIORITY_TVERIFY;
extern int PRIORITY_TARENA;
extern int PRIORITY_TCOMPUTEPOS;

#endif	/* GLOBAL_H */

