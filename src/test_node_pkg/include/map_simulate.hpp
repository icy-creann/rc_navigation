#include <math.h>
#include <vector>
#include <cstdlib>
#define dt 0.01


typedef struct Agent
{
    float x;
    float y;
    float yaw;
}Agent;


void agentUpdate(Agent &agent)
{
    if(agent.x<0||agent.x>12||agent.y<0||agent.y>8)
        agent.yaw+=0.1;
    float vx=cos(agent.yaw),vy=sin(agent.yaw);
    agent.x+=vx*dt;
    agent.y+=vy*dt;
}
