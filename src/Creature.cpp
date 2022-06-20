#include "Creature.h"
#include <windows.h>
#include <iostream>
using namespace std;


Creature::Creature (b2World* world) : m_world (world), m_hasFallen(false), m_isTracking(false) { // Constructeur

		// Creation des corps rigides
		// ==========================

		// Proprietes communes
		b2BodyDef bodyDef;
		bodyDef.fixedRotation = false;
		bodyDef.allowSleep = false;
		bodyDef.awake = true;
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearDamping = 0.01f;
		bodyDef.angularDamping = 0.01f;
        b2PolygonShape shape;
		b2FixtureDef fixture;
		fixture.shape = &shape;

		// COU
		bodyDef.position.Set(1.0f,4.70f);
		m_bodies[COU] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.4f);
        fixture.density = 2.5f;
		m_bodies[COU]->CreateFixture(&fixture);

		// TETE
		bodyDef.position.Set(1.7f,4.7f);
		m_bodies[TETE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.6f, 0.3f);
        fixture.density = 3.5f;
        m_bodies[TETE]->CreateFixture(&fixture);

		// PIED
		bodyDef.position.Set(-1.0f,1.22f);
		m_bodies[PIED] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.7f, 0.2f);
        fixture.density = 10.5f;
        fixture.friction = 0.92;
		m_bodies[PIED]->CreateFixture(&fixture);

		// QUEUE
		bodyDef.position.Set(-1.8f,4.0f);
		m_bodies[QUEUE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.6f, 0.1f);
        fixture.density = 8.50f;
		m_bodies[QUEUE]->CreateFixture(&fixture);

		// PIED 2
		bodyDef.position.Set(1.0f,1.22f);
		m_bodies[PIED2] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.7f, 0.2f);
        fixture.density = 10.5f;
        fixture.friction = 0.92;
		m_bodies[PIED2]->CreateFixture(&fixture);

		// JAMBE
		bodyDef.position.Set(-1.0f,2.25f);
		m_bodies[JAMBE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.8f);
        fixture.density = 5.0f;
		m_bodies[JAMBE]->CreateFixture(&fixture);

		// JAMBE 2
		bodyDef.position.Set(1.0f,2.25f);
		m_bodies[JAMBE2] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.8f);
        fixture.density = 5.0f;
		m_bodies[JAMBE2]->CreateFixture(&fixture);

		// TRONC
		bodyDef.position.Set(0.0f,3.67f);
		m_bodies[TRONC] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(1.2f, 0.6f);
        fixture.density = 2.52f;
		m_bodies[TRONC]->CreateFixture(&fixture);

		// Creation des articulations
		// ==========================

		// Proprietes communes
		b2RevoluteJointDef jointDef;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = 0.5f * b2_pi;
		jointDef.enableLimit = true;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 100.0f;

		// CHEVILLE
		jointDef.Initialize(m_bodies[PIED],m_bodies[JAMBE],m_bodies[PIED]->GetWorldCenter()+b2Vec2(0.0f,0.2f));
		m_joints[CHEVILLE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // CHEVILLE 2
		jointDef.Initialize(m_bodies[PIED2],m_bodies[JAMBE2],m_bodies[PIED2]->GetWorldCenter()+b2Vec2(0.0f,0.2f));
		m_joints[CHEVILLE2] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		// HANCHE JAMBE
		jointDef.Initialize(m_bodies[JAMBE],m_bodies[TRONC],m_bodies[JAMBE]->GetWorldCenter()+b2Vec2(0.0f,0.8f));
		m_joints[HANCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // HANCHE JAMBE2
        jointDef.Initialize(m_bodies[JAMBE2],m_bodies[TRONC],m_bodies[JAMBE2]->GetWorldCenter()+b2Vec2(0.0f,0.8f));
		m_joints[HANCHE2] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // HANCHE COU
		jointDef.Initialize(m_bodies[COU],m_bodies[TRONC],m_bodies[COU]->GetWorldCenter()+b2Vec2(0.0f,-0.4f));
		m_joints[TRONCOU] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // HANCHE TETE
		jointDef.Initialize(m_bodies[TETE],m_bodies[COU],m_bodies[TETE]->GetWorldCenter()+b2Vec2(0.0f,0.0f));
		m_joints[COUTETE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // HANCHE QUEUE
		jointDef.Initialize(m_bodies[QUEUE],m_bodies[TRONC],m_bodies[QUEUE]->GetWorldCenter()+b2Vec2(0.6f,0.00f));
		m_joints[TRONCQUEUE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		// Controleur
		// ==========
		m_PDControllers[CHEVILLE] = new PDController(2.0,1.0);
        m_PDControllers[CHEVILLE2] = new PDController(2.0,1.0);
        m_PDControllers[HANCHE] = new PDController(2.0,1.0);
        m_PDControllers[HANCHE2] = new PDController(2.0,1.0);
        m_PDControllers[TRONCOU] = new PDController(2.0,1.0);
        m_PDControllers[COUTETE] = new PDController(2.0,1.0);
        m_PDControllers[TRONCQUEUE] = new PDController(2.0,1.0);

		// Creation du mouvement de reference a suivre
		m_motion = new Motion("extension2.txt");
}

Creature::~Creature() { // Destructor
		for (int i = 0; i < NB_ARTICULATIONS; ++i) {
			delete m_PDControllers[i]; m_PDControllers[i] = NULL;
		}
		delete m_motion;
}

 void Creature::deleteTete(){

    m_world->DestroyJoint(m_joints[COUTETE]);
    m_bodies[TETE]->DestroyFixture(m_bodies[TETE]->GetFixtureList());
    m_world->DestroyBody(m_bodies[TETE]);
    delete m_PDControllers[COUTETE];
    m_PDControllers[COUTETE] = NULL;
}
void Creature::update(bool balanceControl, bool motionTracking) {

    // Remise a zero des moments articulaires
    for (int j = 0; j < NB_ARTICULATIONS; ++j) m_motorTarget[j] = 0.0;
    // Calcule et met a jour les moments articulaires necessaires a equilibrer la creature
    if (balanceControl) balanceController();

    if (motionTracking) motionTracker();
    else m_isTracking = false;
    // Applique les moments articulaires en tant que "motor speed"
    for (int j = 0; j < NB_ARTICULATIONS; ++j) m_joints[j]->SetMotorSpeed(m_motorTarget[j]);
}

void Creature::balanceController() {

	// BALANCE CONTROLLER
	// ==================

	// Calcul du CdM dans le repere du monde
	m_positionCOM = computeCenterOfMass();

	// Mettre a jour la pose seulement si la creature est debout
	if (m_hasFallen) return;

	// Etape 1.1: Decrire le CPS dans le repere du monde
	// definir la position du CPS localement a un corps rigide bien choisi
	// utiliser b2Body::GetTransform pour optenir la transformation decrivant position+orientation du corps rigide dans le monde
	// utiliser la transformation pour convertir la position locale du CPS en sa position globale

    b2Vec2 cps1 = b2Mul(m_bodies[PIED]->GetTransform(),b2Vec2(0.0f,-0.2f));
	b2Vec2 cps2 = b2Mul(m_bodies[PIED2]->GetTransform(), b2Vec2(0.0f, -0.2f));

	b2Vec2 cpsMonde = b2Vec2((cps1.x + cps2.x) / 2.0f,(cps1.y + cps2.y) / 2.0f);
	// Etape 1.2: Calculer la force d'equilibre, en 3D, a appliquer a partir de la distance entre les CPS et CdM projetes au sol
	b2Vec3 forceToApply (cpsMonde.x - m_positionCOM.x,0.0f,0.0f);

	// Etape 2.1: decrire la position, en 3D, de la cheville dans le repere du pied
	b2Vec3 positionOfAxisLocal(0.0f,0.2f,0.0f);
	// Etape 2.2: decrire l'axe de rotation, en 3D, de la cheville dans le repere du pied : z
    b2Vec3 axisOfRotationLocal (0.0f,0.0f,1.0f);
    // Etape 2.3: appeller jacobianTranspose afin d'estimer le moment necessaire a la cheville pour simuler l'effet de la force
    float jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),PIED,forceToApply);
    // Etape 2.4: mettre a l'echelle l'erreur en moment articulaire
    jointTorque *= 1.0f;
    // Etape 2.5: ajouter le resultat dans le tableau m_motorTarget

    m_motorTarget[CHEVILLE] += jointTorque;
    jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),PIED2,forceToApply);
    m_motorTarget[CHEVILLE2] += jointTorque;

    // Etape 3.1 to 3.5: faire de meme pour la hanche
	positionOfAxisLocal = b2Vec3(0.0f,0.0f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,0.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),JAMBE,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[HANCHE] += jointTorque;

    positionOfAxisLocal = b2Vec3(0.0f,0.0f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,0.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),JAMBE2,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[HANCHE2] += jointTorque;

    positionOfAxisLocal = b2Vec3(0.0f,1.0f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),QUEUE,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[TRONCQUEUE] += jointTorque;

    /*
    positionOfAxisLocal = b2Vec3(0.0f,1.0f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),COU,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[TRONCOU] += jointTorque;
    */

}

void Creature::motionTracker() {

    // Activation du suivi
    if (!m_isTracking) {m_isTracking=true;m_motion->setStartTime(GetTickCount());}
    // Calculer le temps depuis le debut du suivi (en ms)
    DWORD elapsedTime = GetTickCount() - m_motion->getStartTime();
    // Calculer l'indice dans le mouvement
    unsigned int frameIndex = (unsigned int)(elapsedTime / (100 * m_motion->getFrequency())) % m_motion->getNbFrames();
    // Recuperer les donnees correspondant a l'indice
    std::vector<float> frameData = m_motion->getMotionDataAtFrame(frameIndex);
    // Calculer le moment pour chaque articulation
    for (int j = 0; j < NB_ARTICULATIONS; ++j) {
        // Lire l'angle pour l'articulation j
        float targetAngle = frameData[j];
        // Affecter l'angle cible pour le regulateur PD
        m_PDControllers[j]->setTarget(targetAngle);
        // Lire l'angle actuel
        float32 currentAngle = m_joints[j]->GetJointAngle();
        // Calculer le moment (appel a PDController::compute)
        float trackingMotor = m_PDControllers[j]->compute(currentAngle);
        // Ajouter le resultat dans le tableau m_motorTarget
        m_motorTarget[j] += trackingMotor;
    }

}

bool Creature::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // vrai si deja a terre
	if (m_bodies[TRONC]->GetWorldCenter().y < 1.22 ) {
            // detection que la creature est tombee (le CdM du tronc est au niveau de la plateforme)
            for (int j = 0; j < NB_ARTICULATIONS; ++j) {
                    m_joints[j]->SetMotorSpeed(0.0f);
                    m_joints[j]->EnableMotor(false);
            }
            m_hasFallen = true;
	}
	return m_hasFallen;
}

b2Vec2 Creature::computeCenterOfMass() {
    float32 total_mass = 0.0f;
    b2Vec2 sum_COMs(0.0f,0.0f);
    for (int i = 0; i < NB_CORPS; ++i) {
        float32 massBody = m_bodies[i]->GetMass();
        b2Vec2 comBody = m_bodies[i]->GetWorldCenter();
        sum_COMs += massBody * comBody;
        total_mass += massBody;
    }
    return (1.0f / total_mass) * sum_COMs;
}

float Creature::jacobianTranspose(b2Vec3 positionOfAxisLocal, b2Vec3 axisOfRotationLocal, b2Vec3 positionApplyForce, int ID_RIGIDBODY, b2Vec3 forceToApply) {
    // Convertir la position locale de l'articulation en position dans le monde
    b2Vec2 positionOfAxisLocal2D (positionOfAxisLocal.x,positionOfAxisLocal.y);
    b2Vec2 positionOfAxisInWorld2D = b2Mul(m_bodies[ID_RIGIDBODY]->GetTransform(),positionOfAxisLocal2D);
    b2Vec3 positionOfAxisInWorld (positionOfAxisInWorld2D.x,positionOfAxisInWorld2D.y,0.0f);
	// Convertir l'orientation de l'axe de rotation locale dans le monde (rien a faire en 2D)
	b2Vec3 axisOfRotationWorld = axisOfRotationLocal;
	// Calculer le moment articulaire le long de l'axe de rotation (formule de la transposee de Jacobienne)
	float jointTorque = b2Dot(b2Cross(axisOfRotationWorld,(positionApplyForce - positionOfAxisInWorld)),forceToApply);
	// Retourner le moment articulaire
	return jointTorque;
}
