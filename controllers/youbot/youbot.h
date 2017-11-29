#include "YoubotWebotsAllInclude.h"
#include <webots/robot.h>


#define NOMBRE_DONNEES_LASER 681

// Défaut 2
#define VITESSE 8


float *g_Objets; // Largeur, Angle, Distance
#define NOMBRE_PARAMETRES_OBJETS 3

float g_Distance360[1022];

int g_Total_Objets = 0;
int g_Compteur_Objets = 0;



void AffichageDonnes(float Donnees[], int n);
void ScanLaser(float *Distance);
void AvancerVersObjet(YouBotBase *Base_Robot);
void TrouverObjet(YouBotBase* Base_Robot);
void AjouterObjet(float Largeur, float Angle, float Distance);
void CalculerObjets();