#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <gtkmm.h>
#include <string>
#define N_IMAGES 10

class GraphicsManager : public Gtk::Window
{
public:
  GraphicsManager(std::string pictures_path);
  virtual ~GraphicsManager();
  void update_graphics(double voltage, double current, double charge, bool connected);

protected:
  virtual void load_pixbufs();

  //signal handlers:
  virtual bool on_drawingarea_expose(GdkEventExpose *event);


  //Member widgets:
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Background;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Numbers;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Blocks;
  Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Connected;
  Gtk::DrawingArea m_DrawingArea;

  guint m_back_width, m_back_height;
  gint m_frame_num;
  std::string pics_path;
};

#endif
