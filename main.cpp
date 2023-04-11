
#include "ForkLiftDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include <pthread.h>
#include <unistd.h>
#include <irrklang/irrKlang.h>

#include <iostream>
#include <semaphore.h>
#include <stdlib.h>

using namespace irrklang;

GLDebugDrawer	gDebugDrawer;

sem_t mutex;
int retval, mutexCount;
const int PLAYING = 0, QUIT = 1;

void* playBGM(void* arg)
{
	ForkLiftDemo *p = (ForkLiftDemo*)arg;
	ISoundEngine *soundEngine = p->getSoundEngine();
	soundEngine->play2D("8bit-BG.wav", true);

	while(true)
	{
		sem_wait(&mutex);
		if(mutexCount == QUIT)
		{
			sem_post(&mutex);	
			soundEngine->drop();
			return NULL;
		}
	}

	sem_post(&mutex);
	if(soundEngine)
		soundEngine->drop();

	return NULL;
}

int main(int argc,char** argv)
{

        ForkLiftDemo* pForkLiftDemo = new ForkLiftDemo;
	mutexCount = PLAYING;
	pthread_t *threads;
	threads = (pthread_t*)malloc(sizeof(pthread_t));
	pthread_create(&threads[0], NULL, playBGM, pForkLiftDemo);

        pForkLiftDemo->initPhysics(); 
	pForkLiftDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        retval = glutmain(argc, argv,1024,768,"Bullet ForkLift Demo. http://www.continuousphysics.com/Bullet/phpBB2/", pForkLiftDemo);
	
	sem_wait(&mutex);
	mutexCount = QUIT;
	sem_post(&mutex);

	pthread_join(threads[0], NULL);
	return retval;
}

