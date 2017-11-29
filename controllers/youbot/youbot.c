/*
 * File:          youbot.c
 * Date:          24th May 2011
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: David Desbiens
 */
#include "YoubotWebotsAllInclude.h"
#include <webots/robot.h>


#define NOMBRE_DONNEES_LASER 681
#define VITESSE 8


float *g_Objets; // Largeur, Angle, Distance
#define NOMBRE_PARAMETRES_OBJETS 3

float g_Distance360[1022];

int g_Total_Objets = 0;
int g_Compteur_Objets = 0;
int g_Bloc[3] = { 1, 0, 2 };



// Affichage des données du tableau
void AffichageDonnes(float Donnees[], int n) {
  int i;
  
  //Affichage données
  for(i = 0; i < n; i++)
    printf("%.4f - %i\n", Donnees[i], i);
}


// Faire un scan avec le laser - Gauche à Droite
void ScanLaser(float *Distance) {
  OpenLaser();
  GetLaserData(&Distance, 0.f, 0.f);
  CloseLaser();
}


// Approche final du robot
void ApprocherObjet(YouBotBase *Base_Robot, YouBotManipulator *Bras_Robot) {
  float Distances[NOMBRE_DONNEES_LASER];
  int Debut, Fin, Proche, X;
  float Angle = 0;
  
  ScanLaser(&Distances);
  

  // Corriger l'angle si rien en avant
  if (Distances[340] == 5.6f) {
    X = 0;
    while(X > 0 && X < NOMBRE_DONNEES_LASER && 
      Distances[340 - X] != 5.6f && Distances[340 + X] != 5.6f)
      X++;
    
    // Calculer angle
    Angle = 240.0 / NOMBRE_DONNEES_LASER;
    if (Distances[340 - X] != 5.6f)
      Angle *= Distances[340 - X];
    else
      Angle *= Distances[340 + X];
    
    reelYB_MoveBaseAngular(Base_Robot, Angle, VITESSE, true, -1);
    ScanLaser(&Distances);
  }


  // Chercher le début de l'objet en face
  Debut = 340;
  while(Debut > -1 && Distances[Debut] != 0.f && Distances[Debut] != 5.6f)
    Debut--;
  Debut++;

  // Chercher la fin de l'objet en face
  Fin = 340;
  while (Fin < 682 && Distances[Fin] != 0.f && Distances[Fin] != 5.6f)
    Fin++;
  Fin--;
  
  // Trouver le point le plus proche
  Proche = Debut;
  for(X = 0; X <= Fin - Debut; X++)
    if (Distances[Debut + X] < Distances[Proche])
      Proche = Debut + X;

  // Calculer angle
  Angle = 240.0 / NOMBRE_DONNEES_LASER * Proche - 120;

  // Corriger l'angle
  reelYB_MoveBaseAngular(Base_Robot, Angle, VITESSE, true, -1);

  // Approche final
  if (Distances[Proche] != 5.6f) {
    reelYB_MoveBaseLongitudinal(Base_Robot, 
      Distances[Proche] * 25, VITESSE, true, -1);

    // Déposer le bloc
    reelYB_ArmSetPosition(Bras_Robot, PositionBrasBoite, VITESSE, true, -1);
    reelYB_GripperOpen(Bras_Robot);

    // Reculer
    reelYB_MoveBaseLongitudinal(Base_Robot, -10, VITESSE,true, -1);
  }
}


// Faire un scan en 360° de gauche vers la droite
void TrouverObjet(YouBotBase* Base_Robot) {
  float Distances[NOMBRE_DONNEES_LASER];
  int Angle = 0, i, rotation = 0;

  free(g_Objets);
  g_Compteur_Objets = 0;
  g_Total_Objets = 0;

  while(Angle < 360) {
    ScanLaser(&Distances);
    
    //Mettre les données dans le tableau de 360°
    for(i = 85; i < NOMBRE_DONNEES_LASER - 85; i++)
      g_Distance360[(i - 85) + 510 * rotation] = Distances[i];

    reelYB_MoveBaseAngular(Base_Robot, 180, VITESSE, true, -1);
    Angle += 180;
    rotation++;
  }
  
  CalculerObjets();
}


//Ajouter un objet
void AjouterObjet(float Largeur, float Angle, float Distance) {
  float *Objets;
  int i;

  // Grossir le tableau
  Objets = g_Objets;
  g_Total_Objets += NOMBRE_PARAMETRES_OBJETS;
  g_Objets = malloc(sizeof(float) * g_Total_Objets);
  
  // Recopier données
  for(i = 0; i < g_Compteur_Objets; i++)
    g_Objets[i] = Objets[i];
  
  free(Objets);
  
  // Ajouter et l'objet
  g_Objets[i] = Largeur;
  g_Objets[i + 1] = Angle;
  g_Objets[i + 2] = Distance;
  
  g_Compteur_Objets += NOMBRE_PARAMETRES_OBJETS;
}


// pour calculer la largeur des objets et les distances
void CalculerObjets() {
  int Debut, Milieu, Fin;
  float Largeur, Angle, Distance;
  int i;
  
  Debut = Milieu = Fin = -1;
  for(i = 0; i < 1022; i++) {
    if (Debut == -1 && g_Distance360[i] != 5.6f && g_Distance360[i] != 0.f)
      Debut = i; // Début d'une forme
    else if (Debut != -1 && (g_Distance360[i] == 5.6f || g_Distance360[i] == 0.f)) {
      Fin = i; // Fin d'une forme dans le vide pour combler la différence avec Debut
      
      // Trouver le milieu
      Milieu = Debut;
      if (g_Distance360[Debut] > g_Distance360[Debut + 1])
        while (Milieu < Fin && g_Distance360[Milieu] > g_Distance360[Milieu + 1])
          Milieu++;
      else
        while (Milieu < Fin && g_Distance360[Milieu] < g_Distance360[Milieu + 1])
          Milieu++;
      
      // Trouver le côté le plus vers le robot
      if (g_Distance360[Debut] > g_Distance360[Fin])
        Debut = Fin;

      // Calculer l'angle
      Angle = 240.0 / NOMBRE_DONNEES_LASER * (Milieu - Debut);
      
      // Calculer la largeur de l'objet - loi des cosinus
      Largeur = sqrt(pow(g_Distance360[Debut], 2) + pow(g_Distance360[Milieu] , 2) -
        2 * g_Distance360[Debut] * g_Distance360[Milieu] * cos(Angle * M_PI / 180.0));
         
      // Calculer l'angle à partir du début
      Angle = 240.0 / NOMBRE_DONNEES_LASER * Milieu;

      // Distance de l'objet
      Distance = g_Distance360[Milieu - (Milieu - Debut) / 2];

      // Ajouter objet
      AjouterObjet(Largeur, Angle, Distance);
      
      Debut = -1; 
    }
  }
}


// Fonction qui affiche la liste d'objet trouvés et retourne le choix
int ChoisirBoite(YouBotBase* Base_Robot) {
  int i, Choix = 0;
  
  // Choisir un objet
  printf("\n\n");
  printf("Choisir un objet\n");
  printf("------------------------------------\n");  
  for(i = 0; i < g_Compteur_Objets; i += NOMBRE_PARAMETRES_OBJETS)
    printf("%i - Largeur: %.3f - Angle: %.1f - Distance: %.3f\n", 
      i / NOMBRE_PARAMETRES_OBJETS, g_Objets[i], g_Objets[i + 1], g_Objets[i + 2]);
  printf("\n");
  printf("Votre choix ?\n");
  
  // Attente du choix
  wb_robot_keyboard_enable(TIME_STEP);
  while (Choix < '0' || Choix >= g_Compteur_Objets / NOMBRE_PARAMETRES_OBJETS + 48) {
    step();    
    Choix = wb_robot_keyboard_get_key();
  }
  wb_robot_keyboard_disable();
  
  return Choix;
}


int main(int argc, char **argv) {
  float Distances[NOMBRE_DONNEES_LASER];
  int i, Bloc;

  //Initialisation de la base
  YouBotBase *Base_Robot=0;
  reelYB_Init(Base_Robot); //Wraper
  Base_Robot=reelYB_BaseInit();
  if(Base_Robot==0)
    return -1;

  //Initialisation du bras
  YouBotManipulator *Bras_Robot=0;
  Bras_Robot=reelYB_ArmInit();
  if(Bras_Robot==0)
      return -1;
  reelYB_GripperInit(Bras_Robot);

  // Rotation pour trouver un objet
  TrouverObjet(Base_Robot);
  
  // Choisir un objet
  i = ChoisirBoite(Base_Robot);
  i = (i - 48) * NOMBRE_PARAMETRES_OBJETS;
  
  // Rotation vers le choix
  reelYB_MoveBaseAngular(Base_Robot, (g_Objets[i + 1] - 90), VITESSE, true, -1);
    
  // Avancer vers le choix
  passive_wait(2);
  reelYB_MoveBaseLongitudinal(Base_Robot, 
    g_Objets[i + 2] * 20, VITESSE, true, -1);
    
  // Prendre le bloc
  reelYB_GripperOpen(Bras_Robot);
  reelYB_ArmSetPosition(Bras_Robot, ArmPositionbloc1[g_Bloc[Bloc]], VITESSE, true, -1);
  reelYB_GripperClose(Bras_Robot);
  
  // Approche de l'objet
  ApprocherObjet(Base_Robot, Bras_Robot);




  // Manipulation par le clavier
  reelYB_MoveArmAndBaseByKeyboard(Base_Robot);

  return 0;
}

