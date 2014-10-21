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
	marcaRefer.tz = -600.f;

	// inicialización para que cuando empiece a buscar la primera vez sea random a derecha o izquierda
	angulo = qrand()*2.f/RAND_MAX-1;
	
	enfocado = false;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
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
		case STATE::PENSAR:
			pensar();
			break;
		case STATE::ACERCARSE:
			acercarse();
			break;
		case STATE::CELEBRAR:
			//qDebug() << "Celebrar";
			celebrar(); break;
		case STATE::IDLE: 
			if(tagslocal.existsId(marcaBusco, datosMarca))
			{
				QVec vector(6);
				vector[0] = datosMarca.tx;
				vector[1] = 0;
				vector[2] = datosMarca.tz;
				vector[3] = 0;
				vector[4] = datosMarca.ry;
				vector[5] = 0;
				addTransformInnerModel("referencia", "camera", vector);
				vectorMarca = inner->transform("world", QVec::vec3(marcaRefer.tx, 0, marcaRefer.tz), "referencia");
				qDebug() << "Punto: x->" << vectorMarca[0] << " z->" << vectorMarca[2];
			}
			//expulsar();
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
	expulsion[0] = 0;
	expulsion[1] = 0;
	try 
	{
		for(auto i:laser_data)
		{
			//qDebug() << "Datos laser: dist->" << i.dist << " angle->" << i.angle;
			j++;
			expulsion[0] = expulsion[0] + sin(i.angle)*(1/i.dist);
			expulsion[1] = expulsion[1] + cos(i.angle)*(1/i.dist);
			choque = true;
		}
		qDebug() << "Expulsion es: x->" << expulsion[0] << " z->" << expulsion[1] << " suma de " << j << "valores";
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
	differentialrobot_proxy->setSpeedBase(0,0);
	
	if (tagslocal.existsId(marcaBusco, datosMarca))
		estado = STATE::AVANZAR;
	else
		estado = STATE::GIRAR;
}

void SpecificWorker::avanzar()
{

	calcularDestino();
	qDebug() << "vector direccion es: "<<vectorBase[0]<<"-"<<vectorBase[2];
	
	expulsar();
	distancia = sqrt(vectorBase[0]*vectorBase[0]+vectorBase[2]*vectorBase[2]);

	if (distancia > 1000)
	{
		angulo = 0.001*vectorBase[0] - expulsion[0]*50; //Para marca pared
		//angulo = 0.001*vectorBase[0] - expulsion[0]*120; //Para punto delante de marca
		velocidad = 0.5*vectorBase[2] - expulsion[1]*5000;
	}
	else
	{
		angulo = 0.001*vectorBase[0];
		velocidad = 0.5*vectorBase[2];
	}
	if (angulo > 0.5)
		angulo = 0.5;
	if (angulo < -0.5)
		angulo = -0.5;
	if (velocidad > 500)
		velocidad = 500;
	/*if((vectorBase[2] > 2000)&&(angulo > 0)){
		angulo = 0.2;
		velocidad = 350;
	}
	if((vectorBase[2] > 2000)&&(angulo < 0)){
		angulo = -0.2;
		velocidad = 350;
	}*/
	qDebug() << "Velocidad->" << velocidad << " Angulo->" << angulo;
	differentialrobot_proxy->setSpeedBase(velocidad, angulo);
	qDebug() << "Distancia restante: " << distancia;
	if((abs(vectorBase[0]) < 50) && (abs(vectorBase[2]) < 50))
		estado = STATE::PENSAR;
}

void SpecificWorker::pensar()
{
	tagslocal.existsId(marcaBusco, datosMarca);
	if(abs(datosMarca.ry)>0.01)
	{
		if(datosMarca.ry < 0)
			differentialrobot_proxy->setSpeedBase(0,-0.2);
		else
			differentialrobot_proxy->setSpeedBase(0,0.2);
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(400, 0);
		estado = STATE::ACERCARSE;
	}
}

void SpecificWorker::acercarse()
{
	if(!tagslocal.existsId(marcaBusco, datosMarca)){
		estado = STATE::CELEBRAR;
	}
}

void SpecificWorker::celebrar()
{
	differentialrobot_proxy->stopBase();
	qDebug() << "He llegado!";
}

void SpecificWorker::calcularDestino()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		qDebug() << "Veo Marca";
		Rot2DC a(datosMarca.ry+M_PI);
		res = a*(QVec::vec2(marcaRefer.tx,marcaRefer.tz) - QVec::vec2(datosMarca.tx,datosMarca.tz));
		QVec vector(6);
		vector[0] = datosMarca.tx;
		vector[1] = 0;
		vector[2] = datosMarca.tz;
		vector[3] = 0;
		vector[4] = datosMarca.ry;
		vector[5] = 0;
		addTransformInnerModel("referencia", "camera", vector);
		vectorMundo = inner->transform("world", QVec::vec3(marcaRefer.tx, 0, marcaRefer.tz), "referencia");
		//qDebug()<<"Calculo marca en: "<< res[0] << "-" << res[1];
		//vectorMundo = inner->transform("world", QVec::vec3(res[0], 0, res[1]), "camera");
		//vectorMundo = inner->transform("world", QVec::vec3(datosMarca.tx, 0, datosMarca.tz), "camera");
		qDebug()<<"Marca en el mundo esta en: "<< vectorMundo[0] << "-" << vectorMundo[2];
		vectorBase = inner->transform("camera", vectorMundo, "world");
		qDebug()<<"Punto donde voy con transform: "<< vectorBase[0] << "-" << vectorBase[2];
		enfocado = true;
	}
	else{
		qDebug() << "A Ciegas";
		qDebug()<<"Marca en el mundo esta en: "<< vectorMundo[0] << "-" << vectorMundo[2];
		vectorBase = inner->transform("base", vectorMundo, "world");
		qDebug()<<"Punto donde voy con transform: "<<vectorBase[0] << "-" << vectorBase[2];
	}
}

void SpecificWorker::addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D)
{//pose6D vector con tx,ty,tx,0,ry,0 del aprilTags parent es el nombre del padre camera y name el nombre que le quiero dar
		InnerModelNode *nodeParent = inner->getNode(parent);
		if( inner->getNode(name) == NULL)
		{
			InnerModelTransform *node = inner->newTransform(name, "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
			nodeParent->addChild(node);
		}
		inner->updateTransformValues(name, pose6D.x(), pose6D.y(), pose6D.z(), pose6D.rx(), pose6D.ry(), pose6D.rz());	
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
