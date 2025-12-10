import java.util.ArrayList;
import java.util.List;
import java.util.HashSet;


import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;


public class AutonomyThree extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private PositionSensor leftMotorSensor;
	private PositionSensor rightMotorSensor;
	private double encoder_unit=159.23;
	private Odometry odometry;	
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;
	private boolean hardStop = false;
	private int oscillationCounter = 0;

	private double lastSeenDepth = 1.0;
	private double lastSeenHorizontal = 0.0;
	private Double sharedX = null; 
	private Double sharedY = null;
	private Double sharedTheta = null;
	private boolean goToBeacon = false;
	private Double targetX = null;
	private Double targetY = null;
	private boolean targetReceived = false;
	private boolean iAmSender = false;
	private int resendTimer = 0;
	
	public AutonomyThree() {
		timeStep = 64;  // set the control time step
				
		odometry = new Odometry(); // to compute a relative position (if needed)
				
		// Sensors initialization 
		// IR distance sensors
		distanceSensor = new DistanceSensor[8];
		String[] sensorNames = {
				"ps0", "ps1", "ps2", "ps3",
				"ps4", "ps5", "ps6", "ps7"
		};

		for (int i = 0; i < 8; i++) {
			distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
			distanceSensor[i].enable(timeStep);
		}

		// Camera
		camera=this.getCamera("camera");
		camera.enable(timeStep);
		camera.recognitionEnable(timeStep);

		//  WiFi communication
		emitter=getEmitter("emitter");
		receiver=getReceiver("receiver");
		receiver.enable(timeStep);

		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);

		// Motor sensors : to compute relative position
		leftMotorSensor = this.getPositionSensor("left wheel sensor");
		rightMotorSensor = this.getPositionSensor("right wheel sensor");
		leftMotorSensor.enable(timeStep);
		rightMotorSensor.enable(timeStep);

		// LEDS
		leds = new LED[10];
		String[] ledsNames = {
				"led0", "led1", "led2", "led3",
				"led4", "led5", "led6", "led7",
				"led8", "led9"
		};
		for (int i = 0; i < 10; i++) {
			leds[i] = this.getLED(ledsNames[i]);
		}
	}
	
	

	/**
	 * The main method of the robot behaviour
	 */	
	public void run() {

    // Initialisation de l'odométrie (position relative du robot)
    initLocalisation();
    odometry.setTheta(0); // Orientation initiale fixée à 0 pour un repère commun

    // Définition des états de la machine à états
    final int EXPLORE      = 0; // Exploration libre de l’environnement
    final int GO_TO_TARGET = 1; // Navigation cognitive vers une cible connue
    final int FINISHED     = 2; // Robot a atteint la cible et devient balise

    int state = EXPLORE; // L’état initial est l’exploration

    // Variables nécessaires au comportement d’évitement d’obstacles
    int turnSteps = 0;      // Nombre de pas restants pour une rotation
    int straightSteps = 0;  // Nombre de pas pour avancer après un évitement
    int backSteps = 0;      // Nombre de pas de recul
    int turnDirection = 0;  // 1 = tourner gauche, -1 = tourner droite

    while (step(timeStep) != -1) { // Boucle principale de contrôle

        // Mise à jour de la localisation odométrique
        localise(false);
        double myX = odometry.getX();       // Position X actuelle
        double myY = odometry.getY();       // Position Y actuelle
        double myTheta = odometry.getTheta(); // Orientation actuelle

        // Vérification des messages reçus via WiFi
        String msg = checkMailBox();
        if (msg != null && msg.startsWith("TARGET_POS")) {

            // Extraction des coordonnées contenues dans le message
            String[] parts = msg.split(":")[1].split(",");
            targetX = Double.parseDouble(parts[0]); // X de la cible (communiqué par un pair)
            targetY = Double.parseDouble(parts[1]); // Y de la cible

            targetReceived = true; // Le robot sait désormais où aller
            iAmSender = false;     // Ce robot n’est pas le détecteur initial

            System.out.println("Message reçu → GO_TO_TARGET : " + targetX + ", " + targetY);

            state = GO_TO_TARGET;  // Passage en navigation cognitive
        }

        // Si ce robot a trouvé la cible → réémission périodique
        if (iAmSender) {
            resendTimer++; // Incrément du compteur interne
            if (resendTimer > 3) { // Réémission tous les 3 cycles
                broadcastMessage("TARGET_POS:" + targetX + "," + targetY);
                resendTimer = 0; // Réinitialisation
            }
        }

        // Détection d'objets par la caméra
        List<CameraRecognitionObject> objects = cameraDetection();
        CameraRecognitionObject target = targetDetected(objects); // Détection spécifique de la cible

        if (target != null) { // Si la cible est visible

            double depth = target.getPosition()[0];   // Distance de la cible par rapport au robot
            double lateral = target.getPosition()[1]; // Position horizontale (gauche/droite)

            // Si la cible est suffisamment proche → objectif atteint
            if (depth < 0.12) {

                move(0,0);          // Arrêt immédiat du robot
                setLED(8, true);    // Activation LED 8 : signal visuel d’atteinte

                // Conversion des coordonnées locales (caméra) vers le repère global du robot
                double x_local = lateral;
                double y_local = depth;

                double x_global = myX + x_local * Math.cos(myTheta) - y_local * Math.sin(myTheta);
                double y_global = myY + x_local * Math.sin(myTheta) + y_local * Math.cos(myTheta);

                targetX = x_global; // Sauvegarde de la position réelle trouvée
                targetY = y_global;

                System.out.println("Position globale trouvée : " + x_global + ", " + y_global);

                // Diffusion de la position aux autres robots
                broadcastMessage("TARGET_POS:" + x_global + "," + y_global);
                iAmSender = true; // Ce robot devient balise principale
                resendTimer = 0;

                state = FINISHED; // Passage à l’état final
                continue;         // Retour boucle principale (évite traitement inutile)
            }

            // Suivi visuel : orientation vers la cible en fonction du décalage latéral
            if (lateral > 0.05)       move(30,70);  // Cible à gauche → tourner à gauche
            else if (lateral < -0.05) move(70,30);  // Cible à droite → tourner à droite
            else                     move(80,80);  // Cible centrée → avancer droit

            continue; // La poursuite visuelle a priorité absolue
        }

        // Si le robot doit naviguer vers une position de cible déjà connue
        if (state == GO_TO_TARGET && targetReceived) {
            moveTowards(targetX, targetY, 80); // Navigation odométrique vers la cible
            // L’évitement reste traité ci-dessous en parallèle
        }

        // Lecture des capteurs infrarouges pour l’évitement
        double[] ir = readDistanceSensorValues();

        boolean front  = ir[0] > 140 || ir[7] > 140; // Détection d’obstacle frontal
        boolean left   = ir[5] > 140;                // Détection obstacle côté gauche
        boolean right  = ir[2] > 140;                // Détection obstacle côté droit

        double leftVal  = Math.max(ir[5], ir[7]);    // Intensité côté gauche
        double rightVal = Math.max(ir[0], ir[2]);    // Intensité côté droit

        // Étape d’avancement forcée après rotation
        if (straightSteps > 0) {
            move(70,70); // Avance rapide
            straightSteps--;
            continue;
        }

        // Étape de recul en cas de collision frontale
        if (backSteps > 0) {
            move(-40,-40); // Recul pour sortir du contact
            backSteps--;
            continue;
        }

        // Rotation en cours
        if (turnSteps > 0) {

            if (turnDirection == 1) move(-60,60); // Rotation gauche
            else                    move(60,-60); // Rotation droite

            turnSteps--;

            if (turnSteps == 0)
                straightSteps = 8; // Avancer légèrement après rotation

            continue;
        }

        // Gestion obstacle frontal : reculer + déterminer côté le moins bloqué
        if (front) {
            backSteps = 2; // Recul initial
            turnDirection = (leftVal < rightVal) ? 1 : -1; // Choix sens de rotation

            int angle = (int)(8 + Math.abs(leftVal - rightVal)/60); // Calcul de la durée de rotation
            if (angle > 14) angle = 14; // Limite max

            turnSteps = angle;
            continue;
        }

        // Gestion obstacle latéral gauche
        if (left) {
            turnDirection = -1;
            int angle = 6 + (int)(leftVal/100);
            if (angle > 12) angle = 12;
            turnSteps = angle;
            continue;
        }

        // Gestion obstacle latéral droit
        if (right) {
            turnDirection = 1;
            int angle = 6 + (int)(rightVal/100);
            if (angle > 12) angle = 12;
            turnSteps = angle;
            continue;
        }

        // État exploration : le robot avance tant qu’il n’a pas d’information utile
        if (state == EXPLORE)
            move(80,80); // Avance à vitesse maximale

        // État final : robot immobile
        if (state == FINISHED)
            move(0,0); // Arrêt total
    }
}



	/**
	 * Initialisation of the computation of the relative position of the robot
	 */
	private void initLocalisation() {		
		step(timeStep);
		odometry.track_start_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		odometry.setX(0);
		odometry.setY(0);
		odometry.setTheta(Math.PI/2);		
	}
	
	
	/**
	 * To call to compute in real time its own relative position 
	 * @param print : true if the relative position must be printed in the console 
	 */
	protected void localise(boolean print) {
		odometry.track_step_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		if(print) {
			Double[] pos=getPosition();
			System.out.println("Position : "+pos[0]+","+pos[1]);
		}
	}

/**
 * Normalise un angle dans l’intervalle [-PI ; +PI]
 */
private double normalizeAngle(double angle) {
    while (angle > Math.PI)  angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
}

	/**
 * Compte les robots visibles dans la caméra (hors le robot lui-même).
 * Les robots reconnus ont le modèle "e-puck".
 */
private int countVisibleRobots(List<CameraRecognitionObject> detected) {
    int count = 0;

    for (CameraRecognitionObject ob : detected) {
        if (ob.getModel().equals("e-puck")) {
            count++;
        }
    }

    return count;
}

	/**
	 * Get the computed relative position of the robot (x;y)
	 * The starting point is always (0;0)
	 * @return
	 */
	protected Double[] getPosition() {
		return new Double[] {odometry.getX(),odometry.getY()};
	}
	
	/**
	 * Move towards a position (xObj;yObj) within the relative coordinate system of the robot
	 * @param xObj
	 * @param yObj
	 * @param left power of the left motor
	 * @param right power of the right motor
	 */
	@SuppressWarnings("unused")
	private void moveTowards(double targetX, double targetY, double maxSpeed) {

    double myX = odometry.getX();
    double myY = odometry.getY();
    double myTheta = odometry.getTheta();

    // calcul direction vers la cible
    double angleToTarget = Math.atan2(targetY - myY, targetX - myX);

    // erreur de cap
    double angleError = angleToTarget - myTheta;

    // normaliser [-pi ; pi]
    while (angleError > Math.PI) angleError -= 2*Math.PI;
    while (angleError < -Math.PI) angleError += 2*Math.PI;

    double Kp = 2.0; // gain rotation
    double turn = Kp * angleError;

    // commande moteur
    double left = maxSpeed - turn;
    double right = maxSpeed + turn;

    move(left, right);
}



	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++)
			psValues[i] = distanceSensor[i].getValue();

		return psValues;
	}

	/**
	 * 
	 * @param left : a value between [-100;100]%
	 * @param right : a value between [-100;100]%
	 */
	protected void move(double left, double right) {

    // clamp inside [-100 ; 100]
    if (left > 100) left = 100;
    if (left < -100) left = -100;
    if (right > 100) right = 100;
    if (right < -100) right = -100;

    double max = 6.28;

    leftMotor.setVelocity(left * max / 100);
    rightMotor.setVelocity(right * max / 100);
}

	
	/**
	 * Switch on / off a LED according to its num ([0;9])
	 * @param num
	 * @param on : true if the LED is to be switched on, 
	 * or false if the LED is to be switched off
	 */
	protected void setLED(int num, boolean on) {
		if(num < 10) {
			leds[num].set(on ? 1 : 0);
		}
	}

	/**
	 * 
	 * @return an empty list if nothing is detected by the camera, 
	 * a list of CameraRecognitionObject otherwise (see https://cyberbotics.com/doc/reference/camera#camera-recognition-object)
	 */
	protected List<CameraRecognitionObject> cameraDetection() {
		ArrayList<CameraRecognitionObject> detected=new ArrayList<>();
		int nb=camera.getRecognitionNumberOfObjects();
		if(nb >0) {
			CameraRecognitionObject[] objects=camera.getRecognitionObjects();
			for(int i=0;i<objects.length;i++) {
				detected.add(objects[i]);
			}
		}
		return detected;
	}

	/**
	 * Look in a List of camera detected objects if the target is one of them 
	 * @param detected: a List of camera detected objects
	 * @return the target (a specific CameraRecognitionObject) or null
	 */
	protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("cible") == 0)
				return ob;
		}
		return null;		
	}

	/**
	 * Look in a List of camera detected objects if other robots are recognized 
	 * @param detected: a List of camera detected objects
	 * @return a List of CameraRecognitionObject representing the other robots
	 */
	protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
		ArrayList<CameraRecognitionObject> robots=new ArrayList<>();
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("e-puck") == 0)
				robots.add(ob);
		}
		return robots;		
	}
	
	/**
	 * 
	 * @param robot another robot detected by the Camera
	 * @return true if this robot has his LED "led8" on, false otherwise
	 */
	@SuppressWarnings("unused")
	private boolean isLightON(CameraRecognitionObject robot) {
		int[] image=camera.getImage();
		boolean detected=false;

		int[] position=robot.getPositionOnImage();
		int width=robot.getSizeOnImage()[0];
		int height=robot.getSizeOnImage()[1];

		int startx=position[0] - (width + 1)/2;
		int starty=position[1] - (height + 1)/2;

		for (int i = 0; i < width; i++) {
			for(int j=0;j< height;j++) {
				int pixel=image[(startx+i)+(camera.getWidth() * (starty+j))];
				if(Camera.pixelGetRed(pixel) >= 254 && 	Camera.pixelGetGreen(pixel) >= 254 && Camera.pixelGetBlue(pixel) < 200) {
					if (detected) return true;
					else detected=true;					
				}
			}
		}
		return false;

	}

	/**
	 * Allows to send a message at all the other robots
	 * @param message
	 */
	protected void broadcastMessage(String message) {
		emitter.send(message.getBytes());
	}

	/**
	 * Check if a message has been received, and flush the pile
	 * @return null if there is no message, a String otherwise
	 */
	protected String checkMailBox() {
		while(receiver.getQueueLength() > 0) {
			byte[] message=receiver.getData();
			receiver.nextPacket();
			if(message != null) {
				return new String(message);	
			}
			else return null;
		}
		return null;
	}


	public static void main(String[] args) {
		AutonomyThree controller = new AutonomyThree();
		controller.run();
	}
	
	private void escapeObstacle() {
   

    // 1. Recul
    move(-40, -40);
    step(200);

    // 2. Grand virage aléatoire
    if (Math.random() < 0.5)
        move(-20, 60);
    else
        move(60, -20);

    step(500);

    // 3. Avance droite pour sortir du piège
    move(50, 50);
    step(400);

    // 4. Reset mémoire d'angle pour éviter de revenir dans le piège
    lastSeenHorizontal = 0;
    oscillationCounter = 0;
}


	/**
	 * Do NOT modify
	 * Private class providing tools to compute a relative position for the robot
	 */
	private class Odometry{
		private double wheel_distance;
		private double wheel_conversion_left;
		private double wheel_conversion_right;
		private double pos_left_prev;
		private double pos_right_prev;
		private double x;
		private double y;
		private double theta;

		private double increments_per_tour = 1000.0;   // from e-puck.org
		private double axis_wheel_ratio = 1.4134;      // from e-puck.org
		private double wheel_diameter_left = 0.0416;   // from e-puck.org
		private double wheel_diameter_right = 0.0416;  // from e-puck.org
		private double scaling_factor = 0.976;         // default is 1

		public Odometry() {
			// TODO Auto-generated constructor stub
		}

		public int track_start_pos(double pos_left, double pos_right) {
			x=0;
			y=0;
			theta =0;

			pos_left_prev=pos_left;
			pos_right_prev=pos_right;

			wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_diameter_left + wheel_diameter_right) / 2;
			wheel_conversion_left = wheel_diameter_left * scaling_factor * Math.PI / increments_per_tour;
			wheel_conversion_right = wheel_diameter_right * scaling_factor * Math.PI / increments_per_tour;

			return 1;
		}

		public void track_step_pos(double pos_left, double pos_right) {
			double delta_pos_left, delta_pos_right;
			double delta_left, delta_right, delta_theta, theta2;
			double delta_x, delta_y;

			delta_pos_left = pos_left - pos_left_prev;
			delta_pos_right = pos_right - pos_right_prev;
			delta_left = delta_pos_left * wheel_conversion_left;
			delta_right = delta_pos_right * wheel_conversion_right;
			delta_theta = (delta_right - delta_left) / wheel_distance;
			theta2 = theta + delta_theta * 0.5;
			delta_x = (delta_left + delta_right) * 0.5 * Math.cos(theta2);
			delta_y = (delta_left + delta_right) * 0.5 * Math.sin(theta2);

			x += delta_x;
			y += delta_y;
			theta += delta_theta;

			if(theta < 0)
				theta +=2 * Math.PI;
			if(theta > 2 * Math.PI)
				theta -=2 * Math.PI;

			pos_left_prev = pos_left;
			pos_right_prev = pos_right;
		}

		public double getX() {
			return x;
		}

		public void setX(double x) {
			this.x = x;
		}

		public double getY() {
			return y;
		}

		public void setY(double y) {
			this.y = y;
		}

		public double getTheta() {
			return theta;
		}

		public void setTheta(double theta) {
			this.theta = theta;
		}
	}
}