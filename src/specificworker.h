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
	typedef std::vector<std::pair <int, bool> > TMarcas;

	//estados posibles del posRobot
	// GIRAR: gira hacia uno de los dos lados
	// GIRANDO: trata de localizar la marca mientras gira
	// PARA: para el robot cuando tienen la marca fijada
	// AVANZAR: va hacia la marca
	// PENSAR: situado en la marca, trata de ajustar el centro de la marca
	// ACERCARSE: se acerca lo maximo posible a la marca
	// CELEBRAR: celebra el exito
	// IDLE: estado para pruebas
	enum class STATE {GIRAR, GIRANDO, PARAR, AVANZAR, PENSAR, ACERCARSE, CENTRARBRAZO, BAJARBRAZO, COGERCAJA
						, CJGIRAR, CJGIRANDO, CJPARAR, CJAVANZAR, CJPENSAR, CJAPROX, CJDEJARCAJA, CELEBRAR, NOQUEDAN, IDLE};
	STATE estado;
	// variables para la marca que busco, la distancia a la marca y la distancia a la que paro
	int marcaBusco, distanciaMarca, distanciaParada, caja;
	// bool para saber si tengo ya datos de la marca
	bool enfocado, marcaFijada;
	// posicion del robot
	RoboCompDifferentialRobot::TBaseState posRobot;
	// variables para giros, velocidad y distancia
	float radGiro, angulo, velocidad, distancia, intervalo;
	InnerModel *inner;
	//vectores de transformacion para mover un punto al mundo, a la base y vector de expulsion
	QVec marcaRefer, vectorMundo, vectorBase, expulsion, vectorSuelo;
	// reloj para las temporizaciones
	QTime reloj;
	int busqueda;
	// boolean que indica si ya tengo la marca a la que voy en memoria o no
	TPose recogido, coger, dejar, cerrarMano, abrirMano, subirCaja, guardoCaja;
	TMarcas marcas;
	
	struct tag
	{
		tag(){};
		tag( int id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_)
		{
			pos.resize(6);
			id = id_;
			pos[0] = tx_*1000; pos[1] = ty_ *1000; pos[2] = tz_*1000; pos[3] = rx_; pos[4] = ry_; pos[5]=rz_;
		}
		QVec getPos()
		{
			return pos;
		}
		int getID()
		{
			return id;
		}
		int id;
		QVec pos;
	};
	struct tagslocalT
	{
		QMutex mutex;
		void update( const RoboCompGetAprilTags::listaMarcas &t)
		{
			QMutexLocker m(&mutex);
			tags.clear();
			for(auto i: t)
			{
				tag myT(i.id, i.tx, i.ty, i.tz, i.rx, i.ry, i.rz);
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
		bool existFirst(tag &tt)
		{
			QMutexLocker m(&mutex);
			for(auto i: tags)
			{
				if (i.id >= 10)
				{
					tt=i;
					return true;
				}
			}
			return false;
		}
		std::vector<tag> tags;
	};
	tagslocalT tagslocal, tagslocal1;
	tag datosMarca, datosMarcaBrazo;
	
	
	void posicionBrazo(const TPose &lista);
	void moverBrazo ( float x, float y, float z, float dist);
	void dibujarCaja ();
	void dibujarCajaSuelo ();
	void calcularSuelo();
	bool comprobarMarca(int id);
	bool ponerMarcaAColocada( TMarcas &lista, int id);
	bool estaEnBusca(int id);
	bool todasColocadas();
	void noQuedan();
	
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
// 	void  newAprilTag0(const tagsList& tags);
// 	void  newAprilTag1(const tagsList& tags);

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
	void centrarBrazo();
	void bajarBrazo();
	void cogerCaja();
	void aproxCaja();
	void dejarCaja();
	void celebrar();
	void calcularDestino();
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
};

#endif