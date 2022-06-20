#ifndef CREATURE_H
#define CREATURE_H

#include "Box2D/Box2D.h"
#include "PDController.h"
#include "Motion.h"

class Creature {

public:
	Creature (b2World* world);               // Constructeur
	virtual	~Creature();                     // Destructeur

	void update(bool balanceControl, bool motionTracking);  // Mise a jour de la creature (controleur)
	void deleteTete();
	bool hasFallen();					                    // Vrai si la creature est tombee

	b2Vec2 getCOM() {return m_positionCOM;}	            // Retourne la position du CdM
    bool headremoved = false;
protected:

	enum {PIED,JAMBE,TRONC,JAMBE2,PIED2,COU, TETE,QUEUE,NB_CORPS};           // Segments de la creature
	enum {CHEVILLE,HANCHE,CHEVILLE2,HANCHE2,TRONCOU,COUTETE,TRONCQUEUE,NB_ARTICULATIONS};    // Articulations de la creature

	b2World             *	m_world;		    		        // Le monde physique
	b2Body			    *	m_bodies[NB_CORPS];                 // Le tableau de corps rigides
	b2RevoluteJoint     *	m_joints[NB_ARTICULATIONS];		    // Le tableau d'articulations
	float                   m_motorTarget[NB_ARTICULATIONS];    // Le tableau de moments articulaires a appliquer
	PDController		*	m_PDControllers[NB_ARTICULATIONS];	// Le tableau de regulateurs PD (un par articulation)

	bool	    m_hasFallen;            // Vrai si la creature est tombee
	bool        m_isTracking;           // Vrai si le suivi est active
	Motion  *   m_motion;               // Le mouvement a suivre
	b2Vec2      m_positionCOM;          // La position du CdM

	b2Vec2 computeCenterOfMass();                               // Calcule la position du CdM de la creature dans le repere du monde
	void balanceController();                                   // Calcule et met a jour les moments articulaires pour equilibrer la creature

    int jumpStep = 0;
	void motionTracker();                                       // Calcule et met a jour les moments articulaires pour suivre le mouvement de reference
	float jacobianTranspose(b2Vec3,b2Vec3,b2Vec3,int,b2Vec3);   // Calcule les moments necessaires a simuler les effets de la force (en 3D)

};

#endif
