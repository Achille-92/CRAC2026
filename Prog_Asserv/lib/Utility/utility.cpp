#include <utility.h>
#include <Arduino.h>
float limit_angle(float variable)
{
  while (variable > (PI))
    variable -= 2.0*PI;
  while (variable <= (-PI))
    variable += 2.0*PI;
  return variable;
}
int signe(float a)
{
  if (a >= 0)
    return 1;
  else
    return -1;
}
int comparer(float valeur_comparer, float valeur_compareur, float delta)
{
  if ((valeur_comparer) > (valeur_compareur - delta) && (valeur_comparer) < (valeur_compareur + delta))
    return 1;
  else
    return 0;
}
int ensemble(float valeur, float delta)
{
  if ((valeur) > (-delta) && (valeur) < (delta))
    return 1;
  else
    return 0;
}
float logistic(float x, float x0, float k)
{
  float sortie = 1.0 / (1.0 + (k * (fabs(x)-x0)));
  return sortie;
}
float logistic_inverse(float x, float x0, float k)
{
  return (1.0 / logistic(x, x0, k));
}