#include <gtk/gtk.h>

#include "utilapp.h"

int
main (int argc, char *argv[])
{
  return g_application_run (G_APPLICATION (util_app_new ()), argc, argv);
}
