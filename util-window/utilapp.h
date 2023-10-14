#ifndef __UTILAPP_H
#define __UTILAPP_H

#include <gtk/gtk.h>


#define UTIL_APP_TYPE (util_app_get_type ())
G_DECLARE_FINAL_TYPE (UtilApp, util_app, UTIL, APP, GtkApplication)

UtilApp     *util_app_new         (void);


#endif /* __UTILAPP_H */
