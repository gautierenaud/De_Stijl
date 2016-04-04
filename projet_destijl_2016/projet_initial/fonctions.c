#include "fonctions.h"

int write_in_queue(RT_QUEUE * msgQueue, void *data, int size);


void compute_position(void * arg) {
    rt_task_set_periodic(NULL, TM_NOW, 600000000);
    DMessage *message;
    
    while(1)
    {
        rt_task_wait_period(NULL);
        //rt_printf("[tcomputepos] - Activation périodique.\n");
        rt_sem_p(&semComputePosition, TM_INFINITE);
        //rt_printf("[tcomputepos] - Computing position !\n");
        
        // Aquire mutex
        //rt_printf("[tcomputepos] - Wait for mutex Pos.\n");
        rt_mutex_acquire(&mutexPosition, TM_INFINITE);
        //rt_printf("[tcomputepos] - Wait for mutex Arena.\n");
        rt_mutex_acquire(&mutexArena, TM_INFINITE);
        //rt_printf("[tcomputepos] - Wait for mutex image.\n");
        rt_mutex_acquire(&mutexImage, TM_INFINITE);
        
        //rt_printf("[tcomputepos] - Entering in cs.\n");
        
        position = d_image_compute_robot_position(image, arena);
        
        rt_sem_v(&semComputePosition);
        
        
        // Sending message
        if (position)
        {
            message = d_new_message();
            if (!message) {
                rt_printf("[tcomputepos] - Impossible de créer un nouveau message.\n");
            }
            d_message_put_position(message, position);

            if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                message->free(message);
            }
        }
        
        // Release mutex
        rt_mutex_release(&mutexImage);
        rt_mutex_release(&mutexArena);
        rt_mutex_release(&mutexPosition);
         

    }
    
    return ;
}


void detect_arena(void * arg) {
    //No new needed for arena as d_image_compute_arena_position does
    while (1) {
        rt_printf("[tarena] - Attente du sémarphore semDetectArena\n");
        rt_sem_p(&semDetectArena, TM_INFINITE);
        rt_printf("[tarena] - Detection de l'arène ! \n");

        rt_mutex_acquire(&mutexArena, TM_INFINITE);
        rt_mutex_acquire(&mutexImage, TM_INFINITE);

        if (image) {
            arena = d_image_compute_arena_position(image);

        } else {
            rt_printf("[tarena] - Erreur, null pointer sur image\n");
        }

        rt_mutex_release(&mutexImage);
        rt_mutex_release(&mutexArena);
    }

    d_arena_free(arena);
    return;
}

void camera_func(void * arg) {
    /* Create var */
    DJpegimage *jpgimg = d_new_jpegimage();
    DMessage *message;

    if (!jpgimg) {
        rt_printf("[Init Camera] - Impossible de créer une nouvelle image jpeg.\n");
    }

    /* Init camera */
    camera->mIndice = 0;
    d_camera_open(camera);

    /* Getting a frame */
    d_camera_print(camera);

    /* Set task periodic */
    rt_printf("[tcamera] - Debut de l'éxecution de periodique à 600 ms\n");
    rt_task_set_periodic(NULL, TM_NOW, 600000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        //rt_printf ("[tcamera] - Activation périodique\n");

        rt_mutex_acquire(&mutexPosition, TM_INFINITE);
        //rt_printf ("[tcamera] - Waiting for arena mutex.\n");
        rt_mutex_acquire(&mutexArena, TM_INFINITE);
        //rt_printf ("[tcamera] - Waiting for image mutex.\n");
        rt_mutex_acquire(&mutexImage, TM_INFINITE);
        //rt_printf ("[tcamera] - Entering in CS.\n");

       
        d_image_release(image);
        d_camera_get_frame(camera, image);
        
        
        DImage *img = d_new_image();
        *img = *image;
        
        // Dessin image
        if(arena)
        {
            rt_printf("[tcamera] - Drawing arena.\n");
            d_imageshop_draw_arena(img, arena);
        }
        
        if (position)
        {
            rt_printf("[tcamera] - Drawing position.\n");
            d_imageshop_draw_position(img, position);
        }

        // Compressing image 
        d_jpegimage_compress(jpgimg, img);
        //d_image_release(img);
        
        // Sending message
        message = d_new_message();
        if (!message) {
            rt_printf("[Init Camera] - Impossible de créer un nouveau message.\n");
        }
        d_message_put_jpeg_image(message, jpgimg);

        if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            message->free(message);
        }

        // Free imgs 
        d_jpegimage_release(jpgimg);
        
        rt_mutex_release(&mutexImage);
        rt_mutex_release(&mutexArena);
        rt_mutex_release(&mutexPosition);
    }


    // Free var
    d_jpegimage_free(jpgimg);
    d_image_free(image);
    d_camera_close(camera);
    d_camera_free(camera);
}

void envoyer(void *arg) {
    DMessage *msg;
    int err;

    while (1) {
        //rt_printf("tenvoyer : Attente d'un message\n");
        if ((err =
                rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage),
                TM_INFINITE)) >= 0) {
            //rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            serveur->send(serveur, msg);
            msg->free(msg);
        } else {
            //rt_printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void connecter(void *arg) {
    int status;
    DMessage *message;

    rt_printf("tconnect : Debut de l'exécution de tconnect\n");

    while (1) {
        rt_printf("tconnect : Attente du sémarphore semConnecterRobot\n");
        rt_sem_p(&semConnecterRobot, TM_INFINITE);
        rt_printf("tconnect : Ouverture de la communication avec le robot\n");
        status = robot->open_device(robot);

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatCommRobot = status;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {

            //status = robot->start_insecurely(robot);
            status = robot->start(robot);
            

            if (status == STATUS_OK) {
                rt_printf("tconnect : Robot démarrer\n");
                rt_sem_v(&semwatchDog);
            }
            else
            {
                rt_sem_v(&semConnecterRobot);
            }
            compteur_dc=0;
        } else{
            compteur_dc++;
            if (compteur_dc >= 3) {
                compteur_dc = 0;
                robot->stop(robot);
                rt_sem_v(&semConnecterRobot);
            }
        }

        message = d_new_message();
        message->put_state(message, status);

        rt_printf("tconnecter : Envoi message\n");
        message->print(message, 100);

        if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            message->free(message);
        }
    }
}

void communiquer(void *arg) {
    DMessage *msg = d_new_message();
    int var1 = 1;
    int num_msg = 0;

    rt_printf("tserver : Début de l'exécution de serveur\n");
    serveur->open(serveur, "8000");
    rt_printf("tserver : Connexion\n");

    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
    etatCommMoniteur = 0;
    rt_mutex_release(&mutexEtat);

    while (var1 > 0) {
        //rt_printf("tserver : Attente d'un message\n");
        var1 = serveur->receive(serveur, msg);
        num_msg++;
        if (var1 > 0) {
            switch (msg->get_type(msg)) {
                case MESSAGE_TYPE_ACTION:
                    rt_printf("tserver : Le message %d reçu est une action\n",
                            num_msg);
                    DAction *action = d_new_action();
                    action->from_message(action, msg);
                    switch (action->get_order(action)) {
                        case ACTION_CONNECT_ROBOT:
                            rt_printf("tserver : Action connecter robot\n");
                            rt_sem_v(&semConnecterRobot);
                            break;

                        case ACTION_FIND_ARENA:
                            rt_printf("tserver : Le message reçu %d est une action detect arena\n",
                                    num_msg);
                            rt_sem_v(&semDetectArena);
                            break;
                            
                        case ACTION_COMPUTE_CONTINUOUSLY_POSITION:
                            rt_printf("tserver : Le message reçu %d est une action compute position\n",
                                    num_msg);
                            rt_sem_v(&semComputePosition);
                            break;
                            
                        case ACTION_STOP_COMPUTE_POSITION:
                            rt_printf("tserver : Le message reçu %d est une action stop compute position\n",
                                    num_msg);
                            rt_sem_p(&semComputePosition, TM_INFINITE);
                            break;
                    }
                    break;

                case MESSAGE_TYPE_MOVEMENT:
                    rt_printf("tserver : Le message reçu %d est un mouvement\n",
                            num_msg);
                    rt_mutex_acquire(&mutexMove, TM_INFINITE);
                    move->from_message(move, msg);
                    move->print(move);
                    rt_mutex_release(&mutexMove);
                    break;
                case MESSAGE_TYPE_MISSION:
                    rt_printf("tserver : Le message reçu %d est une mission\n", num_msg);
                    rt_mutex_acquire(&mutexMove, TM_INFINITE);
                    mission->from_message(mission, msg);
                    mission->print(mission);
                    rt_mutex_release(&mutexMove);
                    break;
            }
        }
    }
}

void deplacer(void *arg) {
    int status = 1;
    int gauche;
    int droite;
    DMessage *message;

    rt_printf("tmove : Debut de l'éxecution de periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        //rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);

        rt_printf("tmove: entree dans tmove\n");
        if (status == STATUS_OK) {
            rt_mutex_acquire(&mutexMove, TM_INFINITE);
            switch (move->get_direction(move)) {
                case DIRECTION_FORWARD:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_LEFT:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
                case DIRECTION_RIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_STOP:
                    gauche = MOTEUR_STOP;
                    droite = MOTEUR_STOP;
                    break;
                case DIRECTION_STRAIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
            }
            rt_mutex_release(&mutexMove);

            rt_printf("tmove : BOUUUUUUUUUUUUUUUUUUUUUUUUUUUGE\n");
            status = robot->set_motors(robot, gauche, droite);

            if (status != STATUS_OK) {
                rt_mutex_acquire(&mutexEtat, TM_INFINITE);
                etatCommRobot = status;
                rt_mutex_release(&mutexEtat);

                message = d_new_message();
                message->put_state(message, status);

                rt_printf("tmove : Envoi message\n");
                if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) <
                        0) {
                    message->free(message);
                }
            }
            
            compteur_dc=0;
        } else{
            compteur_dc++;
        }
    }
}

void batteryLevel(void *arg) {

    DMessage *message;
    int vbat;
    int status = 1;


    rt_printf("tmove : Debut de l'éxecution de periodique à 1s (tbattery)\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_printf("tbattery : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        //status = etatCommRobot;
        //status = etatCommRobot; //A mettre dans verify status
        status =etatCommRobot;
        rt_mutex_release(&mutexEtat);
        

        if (status == STATUS_OK) {

            rt_mutex_acquire(&mutexEtat, TM_INFINITE);
            robot->get_vbat(robot, &vbat);
            battery->set_level(battery, vbat);

            rt_mutex_release(&mutexEtat);

            message = d_new_message();
            message->put_battery_level(message, battery);
            rt_printf("[tbattery] - Envoi message\n");
            if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                message->free(message);
            }
            compteur_dc = 0;
        }
        else{
            compteur_dc++;

        }
    }

}

void verifyConnectStatus(void *arg) { 

    int status;

    rt_printf("tverify : Debut de l'éxecution de periodique à 1s (tverify)\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    rt_printf("tverify: attente du sémaphore de démarrage du watchdog\n");
    rt_sem_p(&semwatchDog, TM_INFINITE);
    rt_printf("tverify: semaphore ok\n");
    
    while (1) {
        rt_printf("[tverify] - entree dans la boucle\n");
        
        rt_task_wait_period(NULL);
        //rt_printf("tverify: mutex en attente\n");
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        //rt_printf("tverify: mutex acquis\n");
        status = robot -> reload_wdt(robot);
        etatCommRobot = status;
        rt_mutex_release(&mutexEtat);
        //rt_printf("tverify: mutex relased\n");


        if (status == STATUS_OK) {
            //rt_printf("tverify: Activation de tverify\n");

            rt_mutex_acquire(&mutexEtat, TM_INFINITE);
            //printf("tverify: reload watchdog\n");
            status = robot->get_status(robot);
            etatCommRobot = status;
            rt_mutex_release(&mutexEtat);
            compteur_dc=0;
        } 
        else{
            //rt_printf("tverify: no watchdog\n");
            compteur_dc++;
            if (compteur_dc >= 10) {
                rt_printf("RIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIP\n");
                compteur_dc = 0;
                robot->stop(robot);
                rt_sem_v(&semConnecterRobot);
            }
        }

    }
}

int write_in_queue(RT_QUEUE * msgQueue, void *data, int size) {
    void *msg;
    int err;

    msg = rt_queue_alloc(msgQueue, size);
    memcpy(msg, &data, size);

    if ((err = rt_queue_send(msgQueue, msg, sizeof (DMessage), Q_NORMAL)) < 0) {
        rt_printf("Error msg queue send: %s\n", strerror(-err));
    }
    rt_queue_free(&queueMsgGUI, msg);

    return err;
}
