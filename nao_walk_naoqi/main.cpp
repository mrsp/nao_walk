/*
 * main.cpp
 *
 *  Created on: Jul 30, 2015
 *      Author: master
 */



#include "LowLevelPlanner.h"



#ifdef _WIN32
# define ALCALL __declspec(dllexport)
#else
# define ALCALL
#endif

extern "C"
{
    ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
    {
        // init broker with the main broker instance
        // from the parent executable
        AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
        AL::ALBrokerManager::getInstance()->addBroker(pBroker);

        // create module instances
        AL::ALModule::createModule<LowLevelPlanner>(pBroker,"LowLevelPlanner" );
        return 0;
    }

    ALCALL int _closeModule()
    {
        return 0;
    }

} // extern "C"
