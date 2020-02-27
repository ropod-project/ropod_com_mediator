#ifndef COM_MEDIATOR_SIM_H
#define COM_MEDIATOR_SIM_H

#include "com_mediator.h"


class ComMediatorSim : public ComMediator
{
protected:
    virtual void publishTaskMessage(const ropod_ros_msgs::Task& task_msg);

public:
    ComMediatorSim(int argc, char**argv);
    virtual ~ComMediatorSim();

    virtual void setupRos();
    virtual void tearDownRos();
};

#endif /* COM_MEDIATOR_SIM_H */
