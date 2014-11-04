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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
/**
       \brief
       @author authorname
*/
class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
	
	typedef std::vector<std::pair <std::string, float> > TPose;

	//estados posibles del posRobot
	// GIRAR: gira hacia uno de los dos lados
	// GIRANDO: trata de localizar la marca mientras gira
	// PARA: para el robot cuando tienen la marca fijada
	// AVANZAR: va hacia la marca
	// PENSAR: situado en la marca, trata de ajustar el centro de la marca
	// ACERCARSE: se acerca lo maximo posible a la marca
	// CELEBRAR: celebra el exito
	// IDLE: estado para pruebas
	enum class STATE {GIRAR, GIRANDO, PARAR, AVANZAR, PENSAR, ACERCARSE, CELEBRAR, GANCHO, IDLE};
	STATE estado;
	// variables para la marca que busco, la distancia a la marca y la distancia a la que paro
	int marcaBusco, distanciaMarca, distanciaParada;
	// bool para saber si tengo ya datos de la marca
	bool enfocado;
	// posicion del robot
	RoboCompDifferentialRobot::TBaseState posRobot;
	// variables para giros, velocidad y distancia
	float radGiro, angulo, velocidad, distancia, intervalo;
	InnerModel *inner;
	//vectores de transformacion para mover un punto al mundo, a la base y vector de expulsion
	QVec vectorMundo, vectorBase, expulsion;
	// reloj para las temporizaciones
	QTime reloj;
	// boolean que indica si ya tengo la marca a la que voy en memoria o no
	TPose recogido;
	TPose coger;
	
	struct infoPos
	{
		float tx, tz;
		infoPos(){};
		infoPos( float tx_, float tz_)
		{
			tx = tx_;
			tz = tz_;
		}
	};
	infoPos marcaRefer;
	
	struct tag
	{
		int id;
		float tx,tz,ry;	
		tag(){};
		tag( int id_, float tx_, float tz_, float ry_)
		{
			tx = tx_*1000; tz = tz_*1000; ry = ry_; id = id_;
		}
	};
	struct tagslocalT
	{
		QMutex mutex;
		void update( const tagsList &t)
		{
			QMutexLocker m(&mutex);
			tags.clear();
			for(auto i: t)
			{
				tag myT(i.id, i.tx, i.tz, i.ry);
				tags.push_back(myT);
			}
		};
		bool existsId(int id_, tag &tt)
		{
			QMutexLocker m(&mutex);
			for(auto i: tags)
			if( i.id == id_)
			{
				tt=i;
				return true;
			}
			return false;
		}
		std::vector<tag> tags;
	};
	tagslocalT tagslocal;
	tag datosMarca;
	void posicionBrazo(const TPose &lista);

public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const tagsList& tags);

public slots:
 	void compute(); 
	bool comprobarChoque();
	void expulsar();
	void girar();
	void girando();
	void parar();
	void avanzar();
	void pensar();
	void acercarse();
	void gancho();
	void celebrar();
	void calcularDestino();
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
};

#endif