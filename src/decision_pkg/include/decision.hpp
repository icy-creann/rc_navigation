#include <vector>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include "definitions.hpp"


#define PLAN 0 //0:normal 1:rush 2:no move 3:only cure
#define MAXHP 600

#define SENTRY_SIZE 0.5 //
#define PRECISION 0.1  //
#define LENGTH 12
#define WIDTH 8  //the size of the field
#define BAN_X 10
#define BAN_Y 3 //our sentry cannot go to the banned zone(敌方泉水)


Pose mygoal;
std::vector<Pose> keypoint;
Pose enemypop,mypop;
std::vector<Target> enemy;

int state_of_center=0;//0:not occupied 1:friend occupied 2:enemy occupied 3:both occupied
std::vector<uint16_t> HPchain;
int nowstate;
Pose transferPose(Pose initPose)  //将世界坐标系转换为机器人坐标系
{
    Pose result;
    result.x = initPose.x - 1;
    result.y = initPose.y - 7;
    return result;
}
Pose inv_transferPose(Pose initPose)  //将机器人坐标系转换为世界坐标系
{
    Pose result;
    result.x = initPose.x + 1;
    result.y = initPose.y + 7;
    return result;
}
class robot 
{
	friend class FSM;
    //you can change these codes
    private:
        static float distance(Pose p1,Pose p2)
        {
            return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
        }
        static Pose disEnemy(Pose s,Pose e,float keep_distance=3.0)//s:self e:enemy
        {
            s = inv_transferPose(s);
            e = inv_transferPose(e);
            float d=distance(s,e);
            //if(d>=2&&d<=4)
            //   return s;
            Pose r;
            r.x=keep_distance*(s.x-e.x)/d+e.x;
            r.y=keep_distance*(s.y-e.y)/d+e.y;  //矢量计算，定位到以敌人为圆心的圆上
            if(r.x<0+SENTRY_SIZE)r.x=0+SENTRY_SIZE;
            else if(r.x>LENGTH-SENTRY_SIZE)r.x=LENGTH-SENTRY_SIZE;

            if(r.y<0+SENTRY_SIZE)r.y=0+SENTRY_SIZE;
            else if(r.y>WIDTH-SENTRY_SIZE)r.y=WIDTH-SENTRY_SIZE;  //留出机器人自身大小的范围，防止出界
            //std::cout<<d<<' '<<r.x<<' '<<r.y<<std::endl;

            if(r.x>BAN_X&&r.y<BAN_Y){r.x=BAN_X;r.y=BAN_Y;}  //防止误入敌方泉水（禁区）
            r = transferPose(r);  
            return r;
        }
	private:
		//actions 四个状态，idel为大圈巡视，defense_idel为中心增益区巡视，attack，cure
		static void idel1()
		{
			printf("idel1\n");
            mygoal=keypoint[2];
            nowstate=IDEL1;
        }
        static void idel2()
		{
			printf("idel2\n");
            mygoal=keypoint[3];
            nowstate=IDEL2;
		}
        static void idel3()
		{
			printf("idel3\n");
            mygoal=keypoint[4];
            nowstate=IDEL3;
		}
        static void idel4()
		{
			printf("idel4\n");
            mygoal=keypoint[1];
            nowstate=IDEL4;
		}
        static void defense_idel1()
		{
			printf("defense_idel1\n");
            mygoal=keypoint[9];
            nowstate=DEFENSE_IDEL1;
        }
        static void defense_idel2()
		{
			printf("defense_idel2\n");
            mygoal=keypoint[10];
            nowstate=DEFENSE_IDEL2;
		}
        static void defense_idel3()
		{
			printf("defense_idel3\n");
            mygoal=keypoint[11];
            nowstate=DEFENSE_IDEL3;
		}
        static void defense_idel4()
		{
			printf("defense_idel4\n");
            mygoal=keypoint[8];
            nowstate=DEFENSE_IDEL4;
		}
		static void attack() 
		{
			printf("attack\n");
            if(1||enemy.size()==1) //检测到敌人则发动进攻
            {
                enemypop.x = enemy[enemy.size()-1].x; //定位到最后一个敌人后面
                enemypop.y = enemy[enemy.size()-1].y;
                mygoal = disEnemy(mypop,enemypop);
            }
            nowstate=ATTACK;
		}
        /*
		static void defense()
		{
			printf("defense\n");
            mygoal=keypoint[6];
            nowstate=DEFENSE;
		}*/
		static void cure()
		{
			printf("cure\n");
            mygoal=keypoint[7];
            nowstate=CURE;
		}
        /*
        static void attack2()
		{
			printf("attack2\n");
		}
		*/
	public:
        //states
		enum State
		{
			IDEL1=0,
            IDEL2,
            IDEL3,
            IDEL4,
			ATTACK,
			//DEFENSE,
			CURE,
            //ATTACK2, //defense->attack2
            //DEFENSE2, //cure->defense2
            DEFENSE_IDEL1,
            DEFENSE_IDEL2,
            DEFENSE_IDEL3,
            DEFENSE_IDEL4
        };
        //events
		enum Events
		{
			FINDENEMY=0,
			MISSENEMY,
            LOWHP,
            HEALTHY,
            //LASTTIME,
            NOT_OCCUPIED,
            FRIEND_OCCUPIED,
            ENEMY_OCCUPIED,
            BOTH_OCCUPIED
		};
	
    //do not change the codes
    public:
		//
		robot(State curstate,Events event,void(*action)(),State nextstate)
		:_curstate(curstate),_event(event),_action(action),_nextstate(nextstate) {}
	private:
		State _curstate;     
		Events _event;       
		void (*_action)();	 
		State _nextstate;    
};

class FSM
{
	public:
		//do not change these codes
		FSM(robot::State curstate=robot::IDEL1):_curstate(curstate)
		{
			initFSMTable();
		}
		
		void transferState(robot::State nextstate)
		{
			_curstate=nextstate;
		}
		
		void handleEvent(robot::Events event)
		{
			robot::State curstate=_curstate;
			void (*action)()=nullptr;
			robot::State nextstate;
			bool flag=false;
			
			for(int i=0;i<_fsmTable.size();i++)
			{
				if(event==_fsmTable[i]->_event && curstate==_fsmTable[i]->_curstate)
				{
					flag=true;
					action=_fsmTable[i]->_action;
					nextstate=_fsmTable[i]->_nextstate;
					break;
				}
			}
			
			if(flag)
			{
				if(action)
					action();
				transferState(nextstate);
			}
		}

	private:
        //you can change these codes
	    void initFSMTable()
    	{
        	//example:_fsmTable.push_back(new robot(robot::IDEL1,robot::MISSENEMY,&robot::idel1,robot::IDEL2)); 
            //                               Current State;        Event;          Action;       Next State

            //assist idel:1->2 2->3 3->4 4->1
            _fsmTable.push_back(new robot(robot::IDEL1,robot::MISSENEMY,&robot::idel1,robot::IDEL2));    	
        	_fsmTable.push_back(new robot(robot::IDEL2,robot::MISSENEMY,&robot::idel2,robot::IDEL3));
            _fsmTable.push_back(new robot(robot::IDEL3,robot::MISSENEMY,&robot::idel3,robot::IDEL4));
            _fsmTable.push_back(new robot(robot::IDEL4,robot::MISSENEMY,&robot::idel4,robot::IDEL1));
            

            //defense idel:1->2 2->3 3->4 4->1
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL1,robot::MISSENEMY,&robot::defense_idel1,robot::DEFENSE_IDEL2));    	
        	_fsmTable.push_back(new robot(robot::DEFENSE_IDEL2,robot::MISSENEMY,&robot::defense_idel2,robot::DEFENSE_IDEL3));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL3,robot::MISSENEMY,&robot::defense_idel3,robot::DEFENSE_IDEL4));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL4,robot::MISSENEMY,&robot::defense_idel4,robot::DEFENSE_IDEL1));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL1,robot::FINDENEMY,&robot::defense_idel1,robot::DEFENSE_IDEL2));    	
        	_fsmTable.push_back(new robot(robot::DEFENSE_IDEL2,robot::FINDENEMY,&robot::defense_idel2,robot::DEFENSE_IDEL3));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL3,robot::FINDENEMY,&robot::defense_idel3,robot::DEFENSE_IDEL4));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL4,robot::FINDENEMY,&robot::defense_idel4,robot::DEFENSE_IDEL1));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL1,robot::FRIEND_OCCUPIED,&robot::defense_idel1,robot::DEFENSE_IDEL2));    	
        	_fsmTable.push_back(new robot(robot::DEFENSE_IDEL2,robot::FRIEND_OCCUPIED,&robot::defense_idel2,robot::DEFENSE_IDEL3));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL3,robot::FRIEND_OCCUPIED,&robot::defense_idel3,robot::DEFENSE_IDEL4));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL4,robot::FRIEND_OCCUPIED,&robot::defense_idel4,robot::DEFENSE_IDEL1));

            //assist idel->defense idel,if NO_OCCUPIED/FRIEND_OCCUPIED
            _fsmTable.push_back(new robot(robot::IDEL1,robot::NOT_OCCUPIED,&robot::defense_idel1,robot::DEFENSE_IDEL2));   
            _fsmTable.push_back(new robot(robot::IDEL2,robot::NOT_OCCUPIED,&robot::defense_idel2,robot::DEFENSE_IDEL3));   
            _fsmTable.push_back(new robot(robot::IDEL3,robot::NOT_OCCUPIED,&robot::defense_idel3,robot::DEFENSE_IDEL4));   
            _fsmTable.push_back(new robot(robot::IDEL4,robot::NOT_OCCUPIED,&robot::defense_idel4,robot::DEFENSE_IDEL1)); 
            _fsmTable.push_back(new robot(robot::IDEL1,robot::FRIEND_OCCUPIED,&robot::defense_idel1,robot::DEFENSE_IDEL2));   
            _fsmTable.push_back(new robot(robot::IDEL2,robot::FRIEND_OCCUPIED,&robot::defense_idel2,robot::DEFENSE_IDEL3));   
            _fsmTable.push_back(new robot(robot::IDEL3,robot::FRIEND_OCCUPIED,&robot::defense_idel3,robot::DEFENSE_IDEL4));   
            _fsmTable.push_back(new robot(robot::IDEL4,robot::FRIEND_OCCUPIED,&robot::defense_idel4,robot::DEFENSE_IDEL1)); 
            
            //defense idel->assist idel,if ENEMY_OCCUPIED
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL1,robot::ENEMY_OCCUPIED,&robot::idel1,robot::IDEL2));   
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL2,robot::ENEMY_OCCUPIED,&robot::idel2,robot::IDEL3));   
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL3,robot::ENEMY_OCCUPIED,&robot::idel3,robot::IDEL4));   
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL4,robot::ENEMY_OCCUPIED,&robot::idel4,robot::IDEL1));  

            //assist idel->attack if the sentry finds any enemy
            _fsmTable.push_back(new robot(robot::IDEL1,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            _fsmTable.push_back(new robot(robot::IDEL2,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            _fsmTable.push_back(new robot(robot::IDEL3,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            _fsmTable.push_back(new robot(robot::IDEL4,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            //PS:if it is defense idel,it wlll not goto ATTACK stage,because the sentry needs to stay in the center.

            //attack->attack(chase)
            _fsmTable.push_back(new robot(robot::ATTACK,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            //attack->idel1 if the enemy is gone
            _fsmTable.push_back(new robot(robot::ATTACK,robot::MISSENEMY,&robot::idel1,robot::IDEL2));
            //attack->defense if the sentry has limited HP and cannot be cured
            //_fsmTable.push_back(new robot(robot::ATTACK,robot::LOWHP,&robot::defense,robot::DEFENSE));
            //attack->cure if the sentry has limited HP and can be cured
            //_fsmTable.push_back(new robot(robot::ATTACK,robot::CURETIME,&robot::cure,robot::CURE));

            //Whenever LOWHP,goto cure
            _fsmTable.push_back(new robot(robot::IDEL1,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::IDEL2,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::IDEL3,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::IDEL4,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL1,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL2,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL3,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::DEFENSE_IDEL4,robot::LOWHP,&robot::cure,robot::CURE));
            _fsmTable.push_back(new robot(robot::ATTACK,robot::LOWHP,&robot::cure,robot::CURE));
            //cure->IDEL,if HEALTHY
            _fsmTable.push_back(new robot(robot::CURE,robot::HEALTHY,&robot::idel1,robot::IDEL1));
        	//idels->defense if the sentry has limited HP and cannot be cured	
        	//_fsmTable.push_back(new robot(robot::IDEL1,robot::LOWHP,&robot::defense,robot::DEFENSE));
            //_fsmTable.push_back(new robot(robot::IDEL2,robot::LOWHP,&robot::defense,robot::DEFENSE));
            //_fsmTable.push_back(new robot(robot::IDEL3,robot::LOWHP,&robot::defense,robot::DEFENSE));
            //_fsmTable.push_back(new robot(robot::IDEL4,robot::LOWHP,&robot::defense,robot::DEFENSE));

            //idels->cure if the sentry has limited HP and can be cured (or the cure time comes)
            //_fsmTable.push_back(new robot(robot::IDEL1,robot::CURETIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL2,robot::CURETIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL3,robot::CURETIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL4,robot::CURETIME,&robot::cure,robot::CURE));

            //cure->defense2 if the sentry is healthy
            //_fsmTable.push_back(new robot(robot::CURE,robot::HEALTHY,&robot::defense,robot::DEFENSE2));
            
            //defense2->other states
            //_fsmTable.push_back(new robot(robot::DEFENSE2,robot::MISSENEMY,&robot::idel1,robot::IDEL2));
            //_fsmTable.push_back(new robot(robot::DEFENSE2,robot::FINDENEMY,&robot::attack,robot::ATTACK));
            //_fsmTable.push_back(new robot(robot::DEFENSE2,robot::LOWHP,&robot::defense,robot::DEFENSE));
            //_fsmTable.push_back(new robot(robot::DEFENSE2,robot::CURETIME,&robot::cure,robot::CURE));
            
            //defense->cure if the sentry has limited HP and can be cured (or the cure time comes)
            //_fsmTable.push_back(new robot(robot::DEFENSE,robot::CURETIME,&robot::cure,robot::CURE));

            //defesne->attack2
            //_fsmTable.push_back(new robot(robot::DEFENSE,robot::FINDENEMY,&robot::attack,robot::ATTACK2));
            //_fsmTable.push_back(new robot(robot::ATTACK2,robot::MISSENEMY,&robot::defense,robot::DEFENSE));
            //_fsmTable.push_back(new robot(robot::ATTACK2,robot::FINDENEMY,&robot::attack,robot::ATTACK2));

            // ->cure when it is lasttime(60s~80s),whatever the sentry is doing
            //_fsmTable.push_back(new robot(robot::IDEL1,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL2,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL3,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::IDEL4,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::DEFENSE,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::ATTACK,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::CURE,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::ATTACK2,robot::LASTTIME,&robot::cure,robot::CURE));
            //_fsmTable.push_back(new robot(robot::DEFENSE2,robot::LASTTIME,&robot::cure,robot::CURE));
    	}
        
    //do not change these codes
	public:
    	robot::State _curstate; 
	private:
    	std::vector<robot*> _fsmTable;  
};


/*

2025
 ________________________________________________
|  C                                             |
|  u  7                                          |
|  r                                             |
|  e            5/4                              |
|              /                                 |
|            /                                   | 
|          /  .11        .10        3            |
|     0 . 1                        /             |
|                                /               |
|         6    .8          .9  /                 |
|                            /              C    |
|                          2                U    |
|                                           R    |
|___________________________________________E____|

*/
class Decision{
    private:
    FSM *fsm;
    public:
    Decision()
    {
        for(int i=0;i<12;i++)
        {
            Pose point;
            switch(i)
            {
                case 0:point.x=2.7;point.y=3.9;break;//road1(from Spawnpoint to Center)
                case 1:point.x=2.9;point.y=3.9;break;             //assist idel pos1
                case 2:point.x=LENGTH-5.2;point.y=WIDTH-6.0;break;//assist idel pos2
                case 3:point.x=LENGTH-2.9;point.y=WIDTH-3.9;break;//assist idel pos3
                case 4:point.x=5.2;point.y=6.0;break;             //assist idel pos4
                case 5:point.x=4.6;point.y=6.0;break;//road2(from Spawnpoint to Center)
                case 6:point.x=3.0;point.y=3.2;break;
                case 7:point.x=0.5;point.y=7.0;break;//Spawnpoint
                case 8:point.x=5.2;point.y=3.1;break;//defense idel pos1 
                case 9:point.x=6.8;point.y=3.1;break;//defense idel pos2
                case 10:point.x=6.8;point.y=4.9;break;//defense idel pos3
                case 11:point.x=5.2;point.y=4.9;break;//defense idel pos4
                default:break;
            }
            //point.x = point.x - WIDTH/2;
            //point.y = point.y - LENGTH/2;
            point = transferPose(point);
            keypoint.push_back(point);
        }
        fsm=new FSM();
    }    
    private:
    bool curetime(int16_t time)
    {
        //return (time<60);
        return (time>=60&&time<=299);
    }
    bool waittime(int16_t time)
    {
        return (time>298);
    }
    bool starttime(int16_t time)//time<300:the battle starts
    {
        return (time>=293&&time<=298);
        //return (time>=100&&time<=298);
    }
    bool lasttime(int16_t time)//time<60:the sentry cannot be cured
    {
        return (time>=60&&time<=80);
    }    
    bool lowHP(int16_t HP,int16_t thres_HP=215)
    {
        return (HP<thres_HP);
    }
    
    bool healthy(int16_t selfHP,int16_t thres_healthy=520)
    {
        //return (selfHP>enemyHP&&selfHP>400);
        return (selfHP>thres_healthy);
    }
    bool equal(Pose p1,Pose p2)
    {
        return (p1.x-p2.x <= PRECISION && p1.x-p2.x >= -PRECISION && 
                p1.y-p2.y <= PRECISION && p1.y-p2.y >= -PRECISION);
    }
    public:
    int aimTargetId;
    Pose targetPosition;
    Pose current_position;
    
    void update(Pose newposition,std::vector<Target> targets,toNUC_t msg)
    {
        //update the current position
        current_position.x=newposition.x;
        current_position.y=newposition.y;

        //choose the aim target
        auto event=robot::FINDENEMY;
        if(msg.stage!=2||waittime(msg.resT))
        { 
            targetPosition=current_position;
            return;
        }
            
        if(starttime(msg.resT))
        {
            targetPosition=keypoint[1];
            mygoal=keypoint[1];
            nowstate=robot::IDEL2;
            return;
        }
        
        if(!equal(current_position,mygoal)&&nowstate!=robot::ATTACK)
        {
            targetPosition=mygoal;
            return;
        }

        mygoal=current_position;

        //test:miss enemy
        //if(msg.resT>120&&msg.resT<150)
        //    targets.clear();
        switch(msg.middleState)
        {
            case 0:event=robot::NOT_OCCUPIED;break;
            case 1:event=robot::FRIEND_OCCUPIED;break;
            case 2:event=robot::ENEMY_OCCUPIED;break;
            default:break;
        }    
        if((nowstate==robot::IDEL1||nowstate==robot::IDEL2||nowstate==robot::IDEL3||nowstate==robot::IDEL4)&&event!=robot::NOT_OCCUPIED&&event!=robot::FRIEND_OCCUPIED)
        {
            if(targets.empty())
            {
                event=robot::MISSENEMY;
                //enemy.clear();
                //std::cout<<114514<<std::endl;
            }    
            else 
            {
                event=robot::FINDENEMY;
                mypop=current_position;
                enemy=targets;
                //std::cout<<1919810<<std::endl;
            }
        }
        
        
        auto selfHP=msg.HPself_div10*10;
        while(HPchain.size()>6)
        {
            HPchain.erase(HPchain.begin());
        }
        HPchain.push_back(selfHP);
        if(lowHP(selfHP))
        {
            event=robot::LOWHP;
            //if(curetime(msg.resT))
            //    event=robot::CURETIME;
        }
            
        //if(HPchain[HPchain.size()-1]<=HPchain[0]&&nowstate==int(robot::CURE))
        if(healthy(selfHP)&&nowstate==int(robot::CURE))
        {
            event=robot::HEALTHY;
            //std::cout<<"healthy!!"<<std::endl;
        }
        /*
        if(lasttime(msg.resT))
        {
            event=robot::LASTTIME;
        }
        */
        /*debug   
        std::cout<<curetime(msg.resT)<<int(HPchain[HPchain.size()-1]<=HPchain[0])<<int(nowstate==int(robot::CURE))<<std::endl;
        for(int i=0;i<HPchain.size();i++)
            std::cout<<HPchain[i]<<' ';
        std::cout<<std::endl;
        
        */ 
        fsm->handleEvent(event);
        targetPosition=mygoal;
        std::cout<<selfHP<<std::endl;
        std::cout<<"es"<<event<<' '<<nowstate<<std::endl; 
    }
};
