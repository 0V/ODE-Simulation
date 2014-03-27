#include<iostream>
#include<list>
#include<ctime>

#include<ode/ode.h>
#include<drawstuff/drawstuff.h>
#include"texturepath.h"
#include"MT.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

using namespace std;

dWorldID world;
dSpaceID space;
dGeomID ground;
list<dJointID> joint;
dJointGroupID contactgroup;
const dReal ANGLE = 0.4;							// ヒンジの曲がる角度
const dReal m = 0.001, r = 0.025, l = 0.1;			// m 質量  r 半径  l 長さ
float xyz[] = { 0.0, 0.0, 2.0 };					// 視点　位置
float hpr[] = { -180, 0.0, 0.0 };					// 視点　方向
dsFunctions fn;										// DrawStuff 用の構造体
static int counter = 0;								// ループ回数カウント用

typedef struct{
	dBodyID body;
	dGeomID geom;
	dReal r, m, l, side[3], target;
}dObject;											// 物体を表す構造体 r:半径 m:質量 l:長さ target:目標の角度

static list<dObject> leg;							// 体のパーツを入れるためのリスト

// Mersenne twister による一様な擬似乱数生成
double getUniformRand(){
	return genrand_real3();
}

// Call back
static void nearCallBack(void *data, dGeomID o1, dGeomID o2){
	static const int N = 10;
	dContact contact[N];

	bool isGround = ((ground == o1) || (ground == o2));

	int nyan = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

	if (isGround){
		for (int i = 0; i < nyan; i++){
			contact[i].surface.mu = 0.5;			// 接触面の摩擦
			contact[i].surface.mode = dContactBounce;
			contact[i].surface.bounce = 0.8;		// 反発係数
			contact[i].surface.bounce_vel = 4.0;	// 反発に必要な最低速度
			dJointID joint = dJointCreateContact(world, contactgroup, &contact[i]);

			dJointAttach(joint, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}

// dObject 構造体のリストの target にランダムな値を代入
static void setRandomTarget(list<dObject> *obj){
	list<dObject>::iterator it = obj->begin();
	for (unsigned int i = 0; i < obj->size(); i++){
		dReal tmp = (getUniformRand()-0.5)*M_PI*ANGLE*2;
		it->target = tmp;
		it++;
	}
}

// list<dObject> 要素のターゲットの値を全て指定した値にする
static void setTarget(list<dObject> *obj,dReal size){
	list<dObject>::iterator it = obj->begin();
	for (unsigned int i = 0; i < obj->size(); i++){
		dReal tmp = size;
		it->target = tmp;
		it++;
	}
}

// カプセルの生成　第二引数に指定した個数生成する
static void makeCapsule(list<dObject> *list, int lengh = 2){
	for (int i = 0; i < lengh; i++){
		dObject obj;
		obj.r = r;
		obj.l = l;
		obj.m = m;
		obj.body = dBodyCreate(world);
		dMass mass;
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, obj.m, 3, obj.r, obj.l);
		dBodySetMass(obj.body, &mass);

		obj.geom = dCreateCapsule(space, obj.r, obj.l);
		dGeomSetBody(obj.geom, obj.body);
		list->push_back(obj);
	}
}

// list 内のすべての物体をヒンジジョイントで連結し、それぞれのJointIDを第二引数に収納する
static void makeSnakebot(list<dObject> *leg, list<dJointID> *jlist){

	static list<dObject>::iterator it = leg->begin(); // イテレータ
	static list<dObject>::iterator it2 = leg->begin(); // イテレータ
	it2++;

	dReal x0 = 0.0, y0 = 0.0, z0 = 3.0;
	dBodySetPosition(it->body, x0, y0, z0);

	for (unsigned int i = 0; i < leg->size() - 1; i++)
	{
		dJointID jobj;
		dBodySetPosition(it2->body, x0, y0, z0 - (it->l));

		jobj = dJointCreateHinge(world, 0);
		dJointAttach(jobj, it->body, it2->body);
		dJointSetHingeAnchor(jobj, x0, y0, z0 - (it->l / 2));
	
		dJointSetHingeAxis(jobj, 1, 1, 1);
		dJointSetHingeParam(jobj, dParamLoStop, -ANGLE*M_PI);
		dJointSetHingeParam(jobj, dParamHiStop, ANGLE*M_PI);
		jlist->push_back(jobj);

		z0 -= it->l;
		it++;
		it2++;
	}
}

// ヒンジを target の角度まで動かす
static void controlHinge(list<dJointID> jobj,list<dObject> target){
	static const dReal kp = 5.0, fmax = 200;
	list<dJointID>::iterator it = jobj.begin();
	list<dObject>::iterator tit = target.begin();
	
	for (unsigned int i = 0; i < jobj.size(); i++){
		dReal tmp = dJointGetHingeAngle(*it);
		dReal u = kp * (tit->target - tmp);

		dJointSetHingeParam(*it, dParamVel, u);
		dJointSetHingeParam(*it, dParamFMax, fmax);

		it++;
		tit++;
	}
}

// シミュレーションスタート時に呼び出される関数
static void start(){
	dsSetViewpoint(xyz, hpr);
}

// 衝突用空間の生成と設定
static void makeSpace(){
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground = dCreatePlane(space, 0, 0, 1, 0);
}

// シミュレーション中にループして呼び出される
static void simLoop(int pause){
	counter++;
	dSpaceCollide(space, 0, nearCallBack);	//衝突計算

	if (counter > 200){						//この値を変えれば蛇が動く間隔を変えられる
		setRandomTarget(&leg);
		counter = 0;
	}

	controlHinge(joint, leg);

	dWorldStep(world, 0.01);
	dJointGroupEmpty(contactgroup);

	dsSetColor(0.44, 0.10, 0.324);				// 色
	for each (dObject var in leg)
	{
		dsDrawCapsule(dBodyGetPosition(var.body), dBodyGetRotation(var.body), var.l, var.r);
	}
}

// DrawStuff の初期設定
static void setDrawStuff(){
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.path_to_textures = "../../drawstuff/textures";
}

int main(int args, char **argv){
	init_genrand((unsigned int)time(NULL));			// 乱数生成用のシード値を設定

	setDrawStuff();
	dInitODE();
	world = dWorldCreate();
	dWorldSetGravity(world, 0.0, 0.0, -9.8);

	makeSpace();
	makeCapsule(&leg, 20);							// 指定した数だけカプセルを作成
	makeSnakebot(&leg, &joint);						// 蛇型にヒンジを取り付ける

	setTarget(&leg, 0);								// ヒンジを曲げる目標の角度を 0 に初期化

	dsSimulationLoop(args, argv, 1200, 800, &fn);
	dWorldDestroy(world);
	dSpaceDestroy(space);
	dCloseODE();
}