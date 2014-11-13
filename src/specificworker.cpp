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
//otro aprilTags en otra pestaña vinculado al otro puerto
SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	//innerModel que utilizamos
	inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorld.xml");
	//ponemos la velocidad del robot a 0
	differentialrobot_proxy->setSpeedBase(0,0);
	//estado inicial del robot
	estado = STATE::GIRAR;
	// distancia a la que me paro
	distanciaParada = 1600;
	// marca que quiero localizar
	// posicion respecto a la marca a la que quiero ir
	marcaRefer.resize(3);
	marcaRefer[0] = 0; marcaRefer[1] = 0; marcaRefer[2] = -1000;
	// si he captado datos de la marca o no
	enfocado = false;
	marcaFijada = false;
	// temporizador
	reloj.start();
	// intervalo para aplicar al reloj aleatorio para que la espera sea aleatoria no un valor fijo
	intervalo = qrand()*2200.f/RAND_MAX + 4000;
	
	// inicialización para que cuando empiece a buscar la primera vez sea random a derecha o izquierda
	angulo = qrand()*2.f/RAND_MAX-1;
	
	marcas.push_back(std::make_pair<int, bool>(10,false));
	marcas.push_back(std::make_pair<int, bool>(11,false));
	marcas.push_back(std::make_pair<int, bool>(12,true));
	
	// configuro una posicion para el brazo hacia atrás recogido
	recogido.push_back(std::make_pair<std::string, float>("shoulder_right_1", 3.14));
	recogido.push_back(std::make_pair<std::string, float>("shoulder_right_2", -1.4));
	recogido.push_back(std::make_pair<std::string, float>("shoulder_right_3", 0));
	recogido.push_back(std::make_pair<std::string, float>("elbow_right", 2.4));
	recogido.push_back(std::make_pair<std::string, float>("wrist_right_1", 0));
	recogido.push_back(std::make_pair<std::string, float>("wrist_right_2", -1));
	recogido.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
	recogido.push_back(std::make_pair<std::string, float>("finger_right_1", 1));
	recogido.push_back(std::make_pair<std::string, float>("finger_right_2", -1));
	
	//configuro una posicion para el brazo para delante para coger cajas
	coger.push_back(std::make_pair<std::string, float>("shoulder_right_1", 0));
	coger.push_back(std::make_pair<std::string, float>("shoulder_right_2", -0.7));
	coger.push_back(std::make_pair<std::string, float>("elbow_right", 1));
	coger.push_back(std::make_pair<std::string, float>("wrist_right_2", 1.1));
	coger.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
	coger.push_back(std::make_pair<std::string, float>("finger_right_1", 0));
	coger.push_back(std::make_pair<std::string, float>("finger_right_2", 0));
	
	//configuro una posicion para el brazo para delante para dejar cajas
	dejar.push_back(std::make_pair<std::string, float>("shoulder_right_1", 0));
	dejar.push_back(std::make_pair<std::string, float>("shoulder_right_2", -0.7));
	dejar.push_back(std::make_pair<std::string, float>("elbow_right", 1));
	dejar.push_back(std::make_pair<std::string, float>("wrist_right_2", 1.3));
	
	//configuro una posicion para subir la caja
	subirCaja.push_back(std::make_pair<std::string, float>("shoulder_right_1", 0));
	subirCaja.push_back(std::make_pair<std::string, float>("shoulder_right_2", -0.7));
	subirCaja.push_back(std::make_pair<std::string, float>("elbow_right", 1));
	subirCaja.push_back(std::make_pair<std::string, float>("wrist_right_2", 1.3));
	subirCaja.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
	
	//configuro una posicion para cerrar la mano
	cerrarMano.push_back(std::make_pair<std::string, float>("finger_right_1", -0.4));
	cerrarMano.push_back(std::make_pair<std::string, float>("finger_right_2", 0.4));
	
	//configuro una posicion para abrir la mano
	abrirMano.push_back(std::make_pair<std::string, float>("finger_right_1", 0));
	abrirMano.push_back(std::make_pair<std::string, float>("finger_right_2", 0));
	
	// configuro una posicion para el brazo hacia atrás recogido
	guardoCaja.push_back(std::make_pair<std::string, float>("shoulder_right_1", 3.14));
	guardoCaja.push_back(std::make_pair<std::string, float>("shoulder_right_2", -1.4));
	guardoCaja.push_back(std::make_pair<std::string, float>("shoulder_right_3", 0));
	guardoCaja.push_back(std::make_pair<std::string, float>("elbow_right", 2.4));
	guardoCaja.push_back(std::make_pair<std::string, float>("wrist_right_1", 0));
	guardoCaja.push_back(std::make_pair<std::string, float>("wrist_right_2", -1));
	guardoCaja.push_back(std::make_pair<std::string, float>("wrist_right_giro", 0));
	
	posicionBrazo(recogido);
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
	inner->updateTransformValues("robot", posRobot.x , 0, posRobot.z, 0, posRobot.alpha, 0);
	
	try
	{
		RoboCompGetAprilTags::listaMarcas tl0, tl1;
		tagslocal.update(getapriltags0_proxy->checkMarcas());
		tagslocal1.update(getapriltags1_proxy->checkMarcas());
	}
	catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	
	
	
	//elegimos el estado del robot y actuamos segun el que este
	switch(estado)
	{
		case STATE::GIRAR:
			qDebug() << "Girar";
			girar(); 
			break;
		case STATE::GIRANDO:
			qDebug() << "Girando";
			girando(); 
			break;
		case STATE::PARAR:
			qDebug() << "Parar";
			parar(); 
			break;
		case STATE::AVANZAR:
			qDebug() << "Avanzar";
			avanzar(); 
			break;
		case STATE::PENSAR:
			qDebug() << "Pensar";
			pensar();
			break;
		case STATE::ACERCARSE:
			qDebug() << "Acercarse";
			acercarse();
			break;
		case STATE::CENTRARBRAZO:
			qDebug() << "Centrar Brazo";
			centrarBrazo(); 
			break;
		case STATE::BAJARBRAZO:
			qDebug() << "Bajar Brazo";
			bajarBrazo();
			break;
		case STATE::COGERCAJA:
			qDebug() << "Bajar Brazo";
			cogerCaja();
			break;
		case STATE::CJGIRAR:
			qDebug() << "Caja Girar";
			girar(); 
			break;
		case STATE::CJGIRANDO:
			qDebug() << "Caja Girando";
			girando(); 
			break;
		case STATE::CJPARAR:
			qDebug() << "Caja Parar";
			parar(); 
			break;
		case STATE::CJAVANZAR:
			qDebug() << "Caja Avanzar";
			avanzar(); 
			break;
		case STATE::CJPENSAR:
			qDebug() << "Caja Pensar";
			pensar();
			break;
		case STATE::CJAPROX:
			qDebug() << "Aproximar caja";
			aproxCaja();
			break;
		case STATE::CJDEJARCAJA:
			qDebug() << "Dejar caja";
			dejarCaja();
			break;
		case STATE::CELEBRAR:
			qDebug() << "Celebrar";
			celebrar(); 
			break;
		case STATE::IDLE: 
// 			estado para las pruebas;
			qDebug() << "Nada";
			break;
	};
}

void SpecificWorker::moverBrazo(float x, float y, float z, float dist)
{
	RoboCompBodyInverseKinematics::Axis axis;
	axis.x = x;
	axis.y = y;
	axis.z = z;
	try{
		bodyinversekinematics_proxy->advanceAlongAxis("ARM", axis, dist);
	}
	catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
}

void SpecificWorker::posicionBrazo(const TPose &lista)
{
	MotorGoalPosition posicion;
	try 
	{
		for(auto i:lista)
		{
			posicion.name = i.first;
			posicion.position = i.second;
			posicion.maxSpeed = 1.f;
			jointmotor_proxy->setPosition(posicion);
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
}

// devuelve false si no la hemos movido, true si la hemos colocado
bool SpecificWorker::comprobarMarca(int id)
{
	qDebug() << "Compruebo si colocada : " << id;
	try 
	{
		for(auto i:marcas)
		{
			if (i.first == id)
				return i.second;
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
}

bool SpecificWorker::estaEnBusca(int id)
{
	qDebug() << "Compruebo si en busca : " << id;
	try 
	{
		for(auto i:marcas)
		{
			if (i.first == id)
				return true;
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
	return false;
}

bool SpecificWorker::ponerMarcaAColocada(int id)
{
	try 
	{
		for(auto i:marcas)
		{
			if (i.first == id){
				i.second = true;
				return true;
			}
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
	return false;
}

bool SpecificWorker::todasColocadas()
{
	try 
	{
		for(auto i:marcas)
		{
			if (!i.second)
				return false;
		}
	} catch (const Ice::Exception &ex) 
	{
		std::cout << ex << std::endl;
	}
	return true;
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
	if(angulo>0)
	{
		try{	
			radGiro = -0.3;
			differentialrobot_proxy->setSpeedBase(0, radGiro);
			if (estado == STATE::GIRAR)
				estado = STATE::GIRANDO;
			else
				estado = STATE::CJGIRANDO;
		}
		catch( const Ice::Exception &ex)
		{ std::cout << ex << std::endl;}
	}
	else
	{
		try{
			radGiro = 0.3;
			differentialrobot_proxy->setSpeedBase(0, radGiro);
			if (estado == STATE::GIRAR)
				estado = STATE::GIRANDO;
			else
				estado = STATE::CJGIRANDO;
		}
		catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	}
}

// busco la marca para dejar de girar cuando la localice
void SpecificWorker::girando()
{
	if (!marcaFijada)
	{
		qDebug() << "Compruebo si veo algo";
		if((tagslocal.existFirst(datosMarca))&&(estaEnBusca(datosMarca.getID()))&&(!comprobarMarca(datosMarca.getID()))){
			marcaBusco = datosMarca.getID();
			marcaFijada = true;
			if (estado == STATE::GIRANDO)
				estado = STATE::PARAR;
			else
				estado = STATE::CJPARAR;
		}
	}
	else{
		if(tagslocal.existFirst(datosMarca)){
			marcaBusco = datosMarca.getID();
			marcaFijada = true;
			if (estado == STATE::GIRANDO)
				estado = STATE::PARAR;
			else
				estado = STATE::CJPARAR;
		}	
	}
}

// paro el robot, si la marca está localizada avanzo, si se ha perdido vuelvo a buscarla
void SpecificWorker::parar()
{
	try{
		differentialrobot_proxy->setSpeedBase(0,0);
		if (tagslocal.existsId(marcaBusco, datosMarca))
		{
			if (estado == STATE::PARAR)
				estado = STATE::AVANZAR;
			else
				estado = STATE::CJAVANZAR;
		}else
		{	
			angulo = angulo * -1;
			if (estado == STATE::PARAR)
				estado = STATE::GIRAR;
			else
				estado = STATE::CJGIRAR;
		}
	}
	catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
}

// avanzo hacia la marca
void SpecificWorker::avanzar()
{
	// calculo la posicion a la que debo moverme
	calcularDestino();
	qDebug() << "vector direccion es: "<<vectorBase[0]<<"-"<<vectorBase[2];
	//compruebo si estoy sobre la marca para parar y acercarme
	switch(estado)
	{
		case STATE::AVANZAR:
		if((abs(vectorBase[0]) < 150) && (abs(vectorBase[2]) < 150))
			estado = STATE::PENSAR;
		break;
		case STATE::CJAVANZAR:
		if((abs(vectorBase[0]) < 250) && (abs(vectorBase[2]) < 250))
			estado = STATE::CJPENSAR;
		break;
	}
	// calculo las fuerzas de repulsion
	expulsar();

	// si la marca esta lejos calculo la expulsion
	// si la marca esta cerca se ignora la expulsion porque las paredes 
	// anulan la fuerza de atraccion al estar muy cerca
	
	qDebug() << "Factor calculado:" << (10000*(expulsion[1]/4000.0f));
	angulo = 0.001*vectorBase[0] - expulsion[0]*35; //Para marca pared
	velocidad = 0.5*vectorBase[2] - expulsion[1]*(10000*(expulsion[1]/4000.0f));
	qDebug() << "Angulo->" << angulo << " Velocidad->" << velocidad;
	// ajusto los angulos y velocidades para evitar giros bruscos y velocidades exageradas
	if (angulo > 0.4)
		angulo = 0.4;
	if (angulo < -0.4)
		angulo = -0.4;
	if (velocidad > 200)
		velocidad = 200;
	// asigno la velocidad y angulo de giro al robot
	try{
		differentialrobot_proxy->setSpeedBase(velocidad, angulo);
	}
	catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
	
}

// giro para ajustar la marca al centro de la camara
void SpecificWorker::pensar()
{
	tagslocal.existsId(marcaBusco, datosMarca);
	qDebug() << "tx marca es ->" << datosMarca.getPos()[0];
	if(abs(datosMarca.getPos()[0])>1)
	{
		try{
			differentialrobot_proxy->setSpeedBase(0, (datosMarca.getPos()[0]/2000));
		}
		catch( const Ice::Exception &ex)
		{ std::cout << ex << std::endl;}
	}
	else
	{
		try{
			differentialrobot_proxy->setSpeedBase(0, 0);
		}
		catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
		if (estado == STATE::PENSAR)
		{	
			posicionBrazo(coger);
			sleep(1);
			qDebug() << "Ajustado";
			estado = STATE::ACERCARSE;
		}
		else
		{
			try{
				differentialrobot_proxy->setSpeedBase(150,0);
				estado = STATE::CJAPROX;
			}
			catch( const Ice::Exception &ex)
			{ std::cout << ex << std::endl;} 
		}
	}
}

//me acerco a la marca hasta estar pegado
void SpecificWorker::acercarse()
{
	if(tagslocal1.existsId(marcaBusco, datosMarcaBrazo))
	{
		try{
			differentialrobot_proxy->setSpeedBase(0, 0);
			estado = STATE::CENTRARBRAZO;
		}
		catch( const Ice::Exception &ex)
		{ std::cout << ex << std::endl;}
	}
	else
		try{
			differentialrobot_proxy->setSpeedBase(50, 0);
		}
		catch( const Ice::Exception &ex)
	{ std::cout << ex << std::endl;}
}

void SpecificWorker::centrarBrazo()
{
	tagslocal1.existsId(marcaBusco, datosMarcaBrazo);
	if(abs(datosMarcaBrazo.getPos()[0])>10)
	{		
		qDebug() << __FUNCTION__ <<  " muevo tx y espero " << datosMarcaBrazo.getPos()[0];
		moverBrazo(1,0,0,datosMarcaBrazo.getPos()[0]*0.8);
		sleep(2);
	}
	else if (abs(datosMarcaBrazo.getPos()[1])>30)
	{
		qDebug() << __FUNCTION__ <<  " muevo ty y espero " << datosMarcaBrazo.getPos()[1];
		moverBrazo(0,1,0,datosMarcaBrazo.getPos()[1]*0.8);
		sleep(2);
	}
	else{
		//configuro una posicion para cerrar la mano
		float giro = datosMarcaBrazo.getPos()[5];
		qDebug() << "giro: " << giro;
		MotorGoalPosition posicion;
		try 
		{
			posicion.name = "wrist_right_giro";
			posicion.position = giro;
			posicion.maxSpeed = 1.f;
			jointmotor_proxy->setPosition(posicion);
			sleep(1);
			estado = STATE::BAJARBRAZO;
		}catch(Ice::Exception &ex){std::cout << ex << std::cout;}
	}
}

void SpecificWorker::bajarBrazo()
{
	moverBrazo(0,0,1,(datosMarcaBrazo.getPos()[2]-200));
	sleep(2);
	estado = STATE::COGERCAJA;
}

void SpecificWorker::cogerCaja()
{
	qDebug() << "Cojo la caja";
// 	try{
// 		bodyinversekinematics_proxy->setFingers(90000.0f);
// 	}
// 	catch( const Ice::Exception &ex)
// 	{ std::cout << ex << std::endl;}
	caja = marcaBusco;
	posicionBrazo(cerrarMano);
	dibujarCaja();
	posicionBrazo(subirCaja);
	sleep(2);
	posicionBrazo(guardoCaja);
	sleep(2);
	ponerMarcaAColocada(marcaBusco);
	marcaBusco = 3;
	estado = STATE::CJGIRAR;
}

void SpecificWorker::dibujarCaja()
{
	try{
		QString marca = "C" + QString::number(caja,10);
		string marcaString = marca.toStdString();
		innermodelmanager_proxy->removeNode(marcaString);
		Pose3D posCaja;
		posCaja.x = -25; posCaja.y = 0; posCaja.z = 80; posCaja.rx = 0; posCaja.ry = 0; posCaja.rz = 0;
		innermodelmanager_proxy->addTransform(marcaString, "static", "arm_right_7", posCaja);
		Plane3D planoCaja;
		planoCaja.px = 0; planoCaja.py = 0; planoCaja.pz = 0; planoCaja.nx = 0; planoCaja.ny = 0; planoCaja.nz = 0; 
		planoCaja.width = 100; planoCaja.height =100; planoCaja.thickness = 100; planoCaja.texture = "/home/robocomp/robocomp/files/innermodel/tar36h11-10.png";
		QString marcaPlano = "Plano" + marca;
		string marcaPlanoString = marcaPlano.toStdString();
		innermodelmanager_proxy->addPlane(marcaPlanoString, marcaString, planoCaja);
	}
	catch (Ice::Exception &ex)
	{std::cout << ex << std::cout;}
}

void SpecificWorker::dibujarCajaSuelo()
{
	try{
		QString cajaCogida = "C" + QString::number(caja,10);
		string cajaCogidaString = cajaCogida.toStdString();
		innermodelmanager_proxy->removeNode(cajaCogidaString);
		Pose3D posCaja;
		posCaja.x = vectorSuelo[0]; posCaja.y = 50; posCaja.z = vectorSuelo[2]; posCaja.rx = 0; posCaja.ry = 0; posCaja.rz = 0;
		innermodelmanager_proxy->addTransform(cajaCogidaString, "static", "world", posCaja);
		Plane3D planoCaja;
		planoCaja.px = 0; planoCaja.py = 0; planoCaja.pz = 0; planoCaja.nx = 0; planoCaja.ny = 0; planoCaja.nz = 0; 
		planoCaja.width = 100; planoCaja.height =100; planoCaja.thickness = 100; planoCaja.texture = "/home/robocomp/robocomp/files/innermodel/tar36h11-10.png";
		QString cajaCogidaPlano = "Plano" + cajaCogida;
		string cajaCogidaPlanoString = cajaCogidaPlano.toStdString();
		innermodelmanager_proxy->addPlane(cajaCogidaPlanoString, cajaCogidaString, planoCaja);
	}
	catch (Ice::Exception &ex)
	{std::cout << ex << std::cout;}
}

void SpecificWorker::aproxCaja()
{
	if (tagslocal.existsId(marcaBusco, datosMarca))
		{
			if (datosMarca.getPos()[2] < 600)
			{		
				differentialrobot_proxy->setSpeedBase(0,0);
				posicionBrazo(dejar);
				sleep(3);
				estado = STATE::CJDEJARCAJA;
			}
		}
}

void SpecificWorker::dejarCaja()
{
	try{
		calcularSuelo();
		qDebug() << "Vector suelo a: " << vectorSuelo;
		moverBrazo(0,0,1,vectorSuelo[1]-60);
		sleep(1);
		posicionBrazo(abrirMano);
		dibujarCajaSuelo();
	}
	catch( const Ice::Exception &ex)
	{std::cout << ex << std::endl;}
	qDebug() << "Dejo la caja";
	sleep(2);
	posicionBrazo(recogido);
	sleep(1);
	if(todasColocadas())
		estado = STATE::CELEBRAR;
	else
	{
		marcaFijada = false;
		estado = STATE::GIRAR;
	}
}

void SpecificWorker::calcularSuelo(){
	try
	{
		vectorSuelo = inner->transform("world", "arm_right_7");
	}catch( Ice::Exception &ex )
	{ std::cout << ex << std::cout;}
}

// paro y celebro que he llegado
void SpecificWorker::celebrar()
{
	differentialrobot_proxy->stopBase();
	qDebug() << "He llegado!";
	qFatal("Se acabo");
}

// calculo el destino al que va el robot
void SpecificWorker::calcularDestino()
{
	// if: 		-veo la marca- 	obtengo los datos de la marca y los actualizo
	// else: 	-a ciegas- 		con los datos de memoria calculo la nueva posicion de la marca respecto al robot
	if(tagslocal.existsId(marcaBusco, datosMarca))
	{
		try
		{
			//qDebug() << "Veo Marca";
			qDebug()<<"Leo aprilTags en: "<< datosMarca.getPos();
			// añado un nodo al innermodel con la posicion de la marca que he visto
			addTransformInnerModel("referencia", "camera", datosMarca.getPos());
			// calculo donde esta en el mundo el punto que esta frente a la marca localizada
			// guardo esta información para utilizarla cuando no la vea
			vectorMundo = inner->transform("world", marcaRefer, "referencia");
			//para ir a la marca directamente
			//vectorMundo = inner->transform("world", QVec::vec3(datosMarca.tx, 0, datosMarca.tz), "robot");
			qDebug()<<"Marca en el mundo esta en: "<< vectorMundo;
			// calculo el vector de atraccion del robot a la marca a la que me dirijo
			vectorBase = inner->transform("camera", vectorMundo, "world");
			qDebug()<<"Punto donde voy con transform: "<< vectorBase;
			enfocado = true;
		}catch( Ice::Exception &ex )
		{ std::cout << ex << std::cout;}
	}
	else{
		try
		{
			//qDebug() << "A Ciegas";
			qDebug()<<"Marca en el mundo esta en: "<< vectorMundo;
			// calculo el vector de atraccion del robot a la marca a la que me dirijo guardada en memoria
			vectorBase = inner->transform("camera", vectorMundo, "world");
			qDebug()<<"Punto donde voy con transform: "<< vectorBase;
		}catch( Ice::Exception &ex )
		{ std::cout << ex << std::cout;}
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
/*
// metodo para actualizar los datos del apriltags
void SpecificWorker::newAprilTag0(const tagsList& tags)
{
	qDebug() << "hola0";
	tagslocal.update(tags);
}

// metodo para actualizar los datos del apriltags
void SpecificWorker::newAprilTag1(const tagsList& tags)
{
	tagslocal1.update(tags);
	qDebug() << "hola1";
	
}*/
