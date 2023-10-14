#ifndef __UTILAPPWIN_H
#define __UTILAPPWIN_H

#include <gtk/gtk.h>
#include "utilapp.h"


#define UTIL_APP_WINDOW_TYPE (util_app_window_get_type ())
G_DECLARE_FINAL_TYPE (UtilAppWindow, util_app_window, UTIL, APP_WINDOW, GtkApplicationWindow)


UtilAppWindow       *util_app_window_new          (UtilApp *app);
void                 util_app_window_open         (UtilAppWindow *win,
                                                   GFile         *file);


#endif /* __UTILAPPWIN_H */
