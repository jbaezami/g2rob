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
	//innerModel que utilizamos
	inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorld.xml");
	//ponemos la velocidad del robot a 0
	differentialrobot_proxy->setSpeedBase(0,0);
	//estado del robot
	estado = STATE::GIRAR;
	// distancia a la que me paro
	distanciaParada = 800;
	// marca que quiero localizar
	marcaBusco = 10;
	// posicion respecto a la marca a la que quiero ir
	marcaRefer.tx = 0.f;
	marcaRefer.tz = -600.f;
	// si he captado datos de la marca o no
	enfocado = false;
	// temporizador
	reloj.start();
	// intervalo para aplicar al reloj aleatorio para que la espera sea aleatoria no un valor fijo
	intervalo = qrand()*2200.f/RAND_MAX + 4000;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	//obtengo los datos de la posicion del robot
	differentialrobot_proxy->getBaseState(posRobot);
	//ajustamos los valores de la base para los calculos de transformacion
	inner->updateTransformValues("base", posRobot.x , 0, posRobot.z, 0, posRobot.alpha, 0);
	
	//elegimos el estado del robot y actuamos segun el que este
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
			//estado para las pruebas;
			//qDebug() << "Nada";
			break;
	};
}

// modulo que comprueba los valores del laser que esten frente al robot (angulo entre 1.2 y -1.2)
// y aquellos que devuelvan un valor de 400 avisa que posible choque
bool SpecificWorker::comprobarChoque(){
	bool choque = false;
	TLaserData laser_data = laser_proxy->getLaserData();
	try 
	{
		for(auto i:laser_data)
		{
			if ((i.dist < 400)&&(i.angle < 1.2)&&(i.angle > -1.2))
			{
				//qDebug() << "Datos laser: " << i.dist << i.angle;
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

// modulo que realiza la inversa de las distancias obtenidas por el laser
// calcula las componentes y las agrega a un vector que define la fuerza de repulsión
// que generan los obstaculos
void SpecificWorker::expulsar()
{
	TLaserData laser_data = laser_proxy->getLaserData();
	int j = 0;
	expulsion[0] = 0;
	expulsion[1] = 0;
	try 
	{
		for(auto i:laser_data)
		{
			if(i.dist < 4000)
			{
				//qDebug() << "Datos laser: dist->" << i.dist << " angle->" << i.angle;
				j++;
				// calculo las componentes x y z de los vectores de repulsion invertidos
				// invertidos para que cuanto mas cerca esté el obstaculo afecte más
				expulsion[0] = expulsion[0] + sin(i.angle)*(1/i.dist);
				expulsion[1] = expulsion[1] + cos(i.angle)*(1/i.dist);
			}
		}
		//qDebug() << "Expulsion es: x->" << expulsion[0] << " z->" << expulsion[1] << " suma de " << j << "valores";
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
}

// modulo que gira a un lado o a otro
void SpecificWorker::girar()
{
	// inicialización para que cuando empiece a buscar la primera vez sea random a derecha o izquierda
	angulo = qrand()*2.f/RAND_MAX-1;
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

// busco la marca para dejar de girar cuando la localice
void SpecificWorker::girando()
{
	if(tagslocal.existsId(marcaBusco, datosMarca))
		estado = STATE::PARAR;
	else
	{
		// si llevo demasiado tiempo girando me muevo a otro lado
		if (intervalo < reloj.elapsed()){
			//qDebug() << "Quiero salir!!!";
		}
	}
}

// paro el robot, si la marca está localizada avanzo, si se ha perdido vuelvo a buscarla
void SpecificWorker::parar()
{
	differentialrobot_proxy->setSpeedBase(0,0);
	
	if (tagslocal.existsId(marcaBusco, datosMarca))
		estado = STATE::AVANZAR;
	else
		estado = STATE::GIRAR;
}

// avanzo hacia la marca
void SpecificWorker::avanzar()
{
	// calculo la posicion a la que debo moverme
	calcularDestino();
	//qDebug() << "vector direccion es: "<<vectorBase[0]<<"-"<<vectorBase[2];
	// calculo las fuerzas de repulsion
	expulsar();
	// calculo la distancia a la que se encuentra la marca
	distancia = sqrt(vectorBase[0]*vectorBase[0]+vectorBase[2]*vectorBase[2]);

	// si la marca esta lejos calculo la expulsion
	// si la marca esta cerca se ignora la expulsion porque las paredes 
	// anulan la fuerza de atraccion al estar muy cerca
	if (distancia > 1000)
	{
		angulo = 0.001*vectorBase[0] - expulsion[0]*35; //Para marca pared
		velocidad = 0.5*vectorBase[2] - expulsion[1]*9000;
	}
	else
	{
		angulo = 0.001*vectorBase[0];
		velocidad = 0.5*vectorBase[2];
	}
	
	// ajusto los angulos y velocidades para evitar giros bruscos y velocidades exageradas
	if (angulo > 0.5)
		angulo = 0.5;
	if (angulo < -0.5)
		angulo = -0.5;
	if (velocidad > 500)
		velocidad = 500;
	//qDebug() << "Velocidad->" << velocidad << " Angulo->" << angulo;
	// asigno la velocidad y angulo de giro al robot
	differentialrobot_proxy->setSpeedBase(velocidad, angulo);
	//qDebug() << "Distancia restante: " << distancia;
	
	//compruebo si estoy sobre la marca para parar y acercarme
	if((abs(vectorBase[0]) < 50) && (abs(vectorBase[2]) < 50))
		estado = STATE::PENSAR;
}

// giro para ajustar la marca al centro de la camara
void SpecificWorker::pensar()
{
	tagslocal.existsId(marcaBusco, datosMarca);
	if(abs(datosMarca.ry)>0.01)
	{
		if(datosMarca.ry < 0)
			differentialrobot_proxy->setSpeedBase(0,-0.12);
		else
			differentialrobot_proxy->setSpeedBase(0,0.12);
	}
	else
	{
		differentialrobot_proxy->setSpeedBase(400, 0);
		estado = STATE::ACERCARSE;
	}
}

//me acerco a la marca hasta estar pegado
void SpecificWorker::acercarse()
{
	if(!tagslocal.existsId(marcaBusco, datosMarca)){
		estado = STATE::CELEBRAR;
	}
}

// paro y celebro que he llegado
void SpecificWorker::celebrar()
{
	differentialrobot_proxy->stopBase();
	qDebug() << "He llegado!";
}

// calculo el destino al que va el robot
void SpecificWorker::calcularDestino()
{
	// if: 		-veo la marca- 	obtengo los datos de la marca y los actualizo
	// else: 	-a ciegas- 		con los datos de memoria calculo la nueva posicion de la marca respecto al robot
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		//qDebug() << "Veo Marca";
		QVec vector(6);
		vector[0] = datosMarca.tx;
		vector[1] = 0;
		vector[2] = datosMarca.tz;
		vector[3] = 0;
		vector[4] = datosMarca.ry;
		vector[5] = 0;
		// añado un nodo al innermodel con la posicion de la marca que he visto
		addTransformInnerModel("referencia", "camera", vector);
		// calculo donde esta en el mundo el punto que esta frente a la marca localizada
		// guardo esta información para utilizarla cuando no la vea
		//vectorMundo = inner->transform("world", QVec::vec3(marcaRefer.tx, 0, marcaRefer.tz), "referencia");
		//para ir a la marca directamente
		vectorMundo = inner->transform("world", QVec::vec3(datosMarca.tx, 0, datosMarca.tz), "camera");
		//qDebug()<<"Calculo marca en: "<< res[0] << "-" << res[1];
		//qDebug()<<"Marca en el mundo esta en: "<< vectorMundo[0] << "-" << vectorMundo[2];
		// calculo el vector de atraccion del robot a la marca a la que me dirijo
		vectorBase = inner->transform("camera", vectorMundo, "world");
		//qDebug()<<"Punto donde voy con transform: "<< vectorBase[0] << "-" << vectorBase[2];
		enfocado = true;
	}
	else{
		//qDebug() << "A Ciegas";
		//qDebug()<<"Marca en el mundo esta en: "<< vectorMundo[0] << "-" << vectorMundo[2];
		// calculo el vector de atraccion del robot a la marca a la que me dirijo guardada en memoria
		vectorBase = inner->transform("base", vectorMundo, "world");
		//qDebug()<<"Punto donde voy con transform: "<<vectorBase[0] << "-" << vectorBase[2];
	}
}

// metodo para la creacion de un nodo temporal nuevo en el innermodel con la posicion de la marca
void SpecificWorker::addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D)
{
	//pose6D vector con tx,ty,tx,0,ry,0 del aprilTags parent es el nombre del padre camera y name el nombre que le quiero dar
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

// metodo para actualizar los datos del apriltags
void SpecificWorker::newAprilTag(const tagsList& tags)
{
	tagslocal.update(tags);
}
