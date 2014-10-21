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
	enum class STATE {GIRAR, GIRANDO, PARAR, AVANZAR, PENSAR, ACERCARSE, CELEBRAR,IDLE};
	STATE estado;
	float radGiro;
	int marcaBusco, marcaFocus;
	bool localizado, enfocado;
	int distanciaMarca;
	int distanciaParada;
	RoboCompDifferentialRobot::TBaseState posRobot;
	float angulo, velocidad, distancia;
	QVec res, res2;
	InnerModel *inner;
	QVec vectorMarca, vectorMundo, vectorBase, expulsion;
	
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

public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void  newAprilTag(const tagsList& tags);

public slots:
 	void compute(); 
	bool comprobarChoque();
	bool expulsar();
	void girar();
	void girando();
	void parar();
	void avanzar();
	void pensar();
	void acercarse();
	void celebrar();
	void calcularDestino();
	void addTransformInnerModel(const QString &name, const QString &parent, const QVec &pose6D);
};

#endif