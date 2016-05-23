#include <gtk/gtk.h>


void
OnGrabToggle                           (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnRawToggle                            (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnSaveImageClicked                     (GtkButton       *button,
                                        gpointer         user_data);

void
OnVideoToggle                          (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
OnChannelApplyClicked                  (GtkButton       *button,
                                        gpointer         user_data);

void
OnColorFileActivate                    (GtkEditable     *editable,
                                        gpointer         user_data);

void
OnColorFileLoad                        (GtkButton       *button,
                                        gpointer         user_data);

void
OnColorFileSave                        (GtkButton       *button,
                                        gpointer         user_data);

void
OnColorFileSaveAs                      (GtkButton       *button,
                                        gpointer         user_data);

gboolean
OnDAreaConfigure                       (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

void
OnColorIDButton                        (GtkButton       *button,
                                        gpointer         user_data);

gboolean
OnColorStatusConfigure                 (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

gboolean
OnDisplayButtonPress                   (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
OnImageButtonPress                     (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
OnImageConfigure                       (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

void
OnColorIDButton                        (GtkButton       *button,
                                        gpointer         user_data);

void
OnVideoToggle                          (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

gboolean
OnCMapConfigure                        (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

gboolean
OnCMapButtonPress                      (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

gboolean
OnCMapMotion                           (GtkWidget       *widget,
                                        GdkEventMotion  *event,
                                        gpointer         user_data);

gboolean
OnStatusBarConfigure                   (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

void
OnStatusBarAdd                         (GtkContainer    *container,
                                        GtkWidget       *widget,
                                        gpointer         user_data);

void
OnStatusBarRealize                     (GtkWidget       *widget,
                                        gpointer         user_data);

void
OnCMapStatusBarRealize                 (GtkWidget       *widget,
                                        gpointer         user_data);

gboolean
OnCMapExpose                           (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data);

void
OnCMapIntensityClicked                 (GtkButton       *button,
                                        gpointer         user_data);

gboolean
OnImageExpose                          (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data);

gboolean
OnKeyPress                             (GtkWidget       *widget,
                                        GdkEventKey     *event,
                                        gpointer         user_data);

void
OnQuit                                 (GtkObject       *object,
                                        gpointer         user_data);

void
OnLoadCalibClicked                     (GtkButton       *button,
                                        gpointer         user_data);

void
OnResetCalibClicked                    (GtkButton       *button,
                                        gpointer         user_data);

void
OnSaveCalibClicked                     (GtkButton       *button,
                                        gpointer         user_data);

void
OnParamsActivate                       (GtkEditable     *editable,
                                        gpointer         user_data);

void
OnParamsLoad                           (GtkButton       *button,
                                        gpointer         user_data);

void
OnParamsSave                           (GtkButton       *button,
                                        gpointer         user_data);

void
OnParamsSaveAs                         (GtkButton       *button,
                                        gpointer         user_data);

void
OnGrabClicked                          (GtkButton       *button,
                                        gpointer         user_data);

void
OnGrabClicked                          (GtkButton       *button,
                                        gpointer         user_data);

gboolean
on_image_area_motion_notify_event      (GtkWidget       *widget,
                                        GdkEventMotion  *event,
                                        gpointer         user_data);

gboolean
on_image_area_button_release_event     (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);
