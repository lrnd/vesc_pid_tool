#include <gtk/gtk.h>

#include "utilapp.h"
#include "utilappwin.h"

struct _UtilApp
{
  GtkApplication parent;
};

G_DEFINE_TYPE(UtilApp, util_app, GTK_TYPE_APPLICATION);

static void
util_app_init (UtilApp *app)
{
}

static void
util_app_startup (GApplication *app)
{
  GtkBuilder *builder;
  G_APPLICATION_CLASS (util_app_parent_class)->startup (app);
  builder = gtk_builder_new_from_resource ("/org/gtk/utilapp/app-menu.ui");

}

static void
util_app_activate (GApplication *app)
{
  UtilAppWindow *win;

  win = util_app_window_new (UTIL_APP (app));
  gtk_window_present (GTK_WINDOW (win));
}

static void
util_app_open (GApplication  *app,
                  GFile        **files,
                  gint           n_files,
                  const gchar   *hint)
{
  GList *windows;
  UtilAppWindow *win;
  int i;

  windows = gtk_application_get_windows (GTK_APPLICATION (app));
  if (windows)
    win = UTIL_APP_WINDOW (windows->data);
  else
    win = util_app_window_new (UTIL_APP (app));

  for (i = 0; i < n_files; i++)
    util_app_window_open (win, files[i]);

  gtk_window_present (GTK_WINDOW (win));
}

static void
util_app_class_init (UtilAppClass *class)
{
  G_APPLICATION_CLASS (class)->activate = util_app_activate;
  G_APPLICATION_CLASS (class)->open = util_app_open;
}

UtilApp *
util_app_new (void)
{
  return g_object_new (UTIL_APP_TYPE,
                       "application-id", "org.gtk.utilapp",
                       "flags", G_APPLICATION_HANDLES_OPEN,
                       NULL);
}
