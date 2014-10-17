/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 #include "specificworker.h"
#include </home/salabeta/robocomp/components/robocomp-ursus-rockin/trajectoryrobot2d/src/specificworker.h>
#include <qt4/QtCore/qdebug.h>
#include <qt4/QtCore/qglobal.h>
#include <math.h>

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	
	inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorld.xml");
	differentialrobot_proxy->setSpeedBase(0,0);
	estado = STATE::GIRAR;
	localizado = false;
	
	// distancia a la que me paro
	distanciaParada = 800;
	
	marcaBusco = 0;
	
	marcaRefer.tx = 0.f;
	marcaRefer.tz = 600.f;

	// inicialización para que cuando empiece a buscar la primera vez sea random a derecha o izquierda
	angulo = qrand()*2.f/RAND_MAX-1;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	/*
	QVec memoria;
	inner->transform("world", QVec::zeros(3), "root").print("robot");
	if(tagslocal.existsId(marcaBusco,datosMarca))
	{
		inner->transform("world", QVec::vec3(datosMarca.tx,datosMarca.tz), "camera").print("tag");
		memoria = inner->transform("world", QVec::vec3(datosMarca.tx, datosMarca.tz), "camera");
		memoria.print("tag");
	}
	else
		QVec target = inner->transform("robot", memoria, "world");
	*/
	differentialrobot_proxy->getBaseState(posRobot);
	inner->updateTransformValues("base", posRobot.x , 0, posRobot.z, 0, posRobot.alpha, 0);
	
	switch(estado)
	{
		case STATE::GIRAR:
			//qDebug() << "Girar";
			girar(); break;
		case STATE::GIRANDO:
			//qDebug() << "Girando";
			girando(); break;
		case STATE::PARAR:
			//qDebug() << "Parar";
			parar(); break;
		case STATE::AVANZAR:
			//qDebug() << "Avanzar";
			avanzar(); break;
		case STATE::AVANZANDO:
			//qDebug() << "Avanzar";
			avanzando(); break;
		case STATE::PENSAR:
			pensar();
			break;
		case STATE::CELEBRAR:
			//qDebug() << "Celebrar";
			celebrar(); 
			break;
		case STATE::IDLE: 
			convertirPuntoEje();
			pasarMarcaAlMundo();
			//qDebug() << "Nada";
			break;
	};
}

bool SpecificWorker::comprobarChoque(){
	bool choque = false;
	TLaserData laser_data = laser_proxy->getLaserData();
	try 
	{
		for(auto i:laser_data)
		{
			if ((i.dist < 400)&&(i.angle < 1.2)&&(i.angle > -1.2))
			{
				qDebug() << "Datos laser: " << i.dist << i.angle;
				differentialrobot_proxy->stopBase();
				choque = true;
				break;
			}
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
	return choque;
}

bool SpecificWorker::expulsar()
{
	bool choque = false;
	TLaserData laser_data = laser_proxy->getLaserData();
	int j = 0;
	expulsion = 0;
	try 
	{
		for(auto i:laser_data)
		{
			if ((i.dist < 400)&&(i.angle < 1.2)&&(i.angle > -1.2))
			{
				j++;
				expulsion = expulsion + sin(i.angle)*i.dist;
				qDebug() << "Datos laser: " << i.dist << i.angle;
				choque = true;
			}
		}
		qDebug() << "Expulsion es: " << expulsion << " suma de " << j << "valores";
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
	return choque;
}

void SpecificWorker::girar()
{
	if(angulo>0)
	{
		radGiro = -0.6;
		differentialrobot_proxy->setSpeedBase(0, radGiro);
		estado = STATE::GIRANDO;
	}
	else
	{
		radGiro = 0.6;
		differentialrobot_proxy->setSpeedBase(0, radGiro);
		estado = STATE::GIRANDO;
	}
}

void SpecificWorker::girando()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
		estado = STATE::PARAR;
}

void SpecificWorker::parar()
{
	differentialrobot_proxy->getBaseState(posRobot);
	differentialrobot_proxy->setSpeedBase(0,0);
	
	if (tagslocal.existsId(marcaBusco, datosMarca))
		estado = STATE::IDLE;
	else
		estado = STATE::GIRAR;
}

void SpecificWorker::avanzar()
{

	
		if (tagslocal.existsId(marcaBusco, datosMarca))
		{
			convertirPuntoEje();
			//pasarMarcaAlMundo();
			vectorMundo = inner->transform("camera", QVec::vec3(datosMarca.tx, 0, datosMarca.tz), "world");
			vectorBase = inner->transform("world", vectorMundo, "base");
		}
		
		if (expulsar() && vectorBase[1] > 1000)
			vectorBase[0] = vectorBase[0] - expulsion;
		angulo = 0.001*vectorBase[0];
		
		if (angulo > 0.7)
			angulo = 0.7;
		velocidad = 0.5*vectorBase[2];
		if (velocidad > 500)
			velocidad = 500;
		if((vectorBase[2] > 2000)&&(angulo > 0)){
			angulo = 0.2;
			velocidad = 350;
		}
		if((vectorBase[2] > 2000)&&(angulo < 0)){
			angulo = -0.2;
			velocidad = 350;
		}
		differentialrobot_proxy->setSpeedBase(velocidad, angulo);
}

void SpecificWorker::pensar()
{
	tagslocal.existsId(marcaBusco, datosMarca);
	if(abs(datosMarca.ry)>0.01)
	{
		if(datosMarca.ry < 0)
			differentialrobot_proxy->setSpeedBase(0,-0.05);
		else
			differentialrobot_proxy->setSpeedBase(0,0.05);
	}
	else
	{
		differentialrobot_proxy->stopBase();
		estado = STATE::AVANZANDO;
	}
}

void SpecificWorker::avanzando()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		differentialrobot_proxy->setSpeedBase(150,0);
	}	
	else
		estado = STATE::CELEBRAR;
}

void SpecificWorker::celebrar()
{
	differentialrobot_proxy->stopBase();
	qDebug() << "He llegado!";
}

void SpecificWorker::convertirPuntoEje()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		Rot2DC a(datosMarca.ry+M_PI);
		res = a*(QVec::vec2(marcaRefer.tx,marcaRefer.tz) - QVec::vec2(datosMarca.tx,datosMarca.tz));
	}
	else
	{
		//estado = STATE::PARAR;
	}
}

void SpecificWorker::pasarMarcaAlMundo()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		differentialrobot_proxy->getBaseState(posRobot);
		Rot2D b(posRobot.alpha);
		res2 = b*QVec::vec2(datosMarca.tx,datosMarca.tz) + QVec::vec2(posRobot.x,posRobot.z);
		//res2 = b*QVec::vec2(datosMarca.tx,datosMarca.tz) + QVec::vec2(posRobot.x,posRobot.z);
		qDebug() << "Robot en el mundo en x: "<<posRobot.x<<" z: "<<posRobot.z << " alpha: " << posRobot.alpha;
		b.print("b");
		qDebug() << "Marca respecto robot x: "<<datosMarca.tx<<" z: "<<datosMarca.tz;
		qDebug() << "Marca en el mundo en x: "<<res2[0]<<" z: "<<res2[1];
	}
	estado = STATE::AVANZAR;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
};

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	tagslocal.update(tags);
}
