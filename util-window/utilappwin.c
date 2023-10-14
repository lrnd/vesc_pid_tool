#include <gtk/gtk.h>

#include "utilapp.h"
#include "utilappwin.h"

struct _UtilAppWindow
{
  GtkApplicationWindow parent;
};

G_DEFINE_TYPE(UtilAppWindow, util_app_window, GTK_TYPE_APPLICATION_WINDOW);

static void
util_app_window_init (UtilAppWindow *win)
{
  gtk_widget_init_template (GTK_WIDGET (win));
}

static void
util_app_window_class_init (UtilAppWindowClass *class)
{
  gtk_widget_class_set_template_from_resource (GTK_WIDGET_CLASS (class),
                                               "/org/gtk/utilapp/window.ui");
}

UtilAppWindow *
util_app_window_new (UtilApp *app)
{
  return g_object_new (UTIL_APP_WINDOW_TYPE, "application", app, NULL);
}

void
util_app_window_open (UtilAppWindow *win,
                         GFile            *file)
{
}
