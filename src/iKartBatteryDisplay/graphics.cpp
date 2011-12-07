#include <math.h>
#include "graphics.h"

#ifndef M_PI
#define M_PI 3.1415927
#endif

using namespace std;

//Called by DemoWindow;
Gtk::Window* do_pixbufs()
{
  fprintf(stderr,"ERROR: program should not reach this point \n");
  return new GraphicsManager("default");
}

GraphicsManager::GraphicsManager(string pictures_path)
{
  m_back_width = 0;
  m_back_height = 0;


  set_title("Battery Monitor");
  set_resizable(false);
  //set_decorated(false);
  set_keep_above(true);
  stick();

  pics_path = pictures_path;
  load_pixbufs();

  set_size_request(m_back_width, m_back_height);
  m_refPixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, FALSE, 8, m_back_width, m_back_height);
  m_DrawingArea.signal_expose_event().connect(sigc::mem_fun(*this, &GraphicsManager::on_drawingarea_expose));
  add(m_DrawingArea);

  show_all();
}

GraphicsManager::~GraphicsManager()
{
}

void GraphicsManager::load_pixbufs()
{
  if(m_refPixbuf_Background)
    return; /* already loaded earlier */

  string filename;
  filename = pics_path+"background.bmp";
  printf ("loading: %s\n", filename.c_str());
  m_refPixbuf_Background = Gdk::Pixbuf::create_from_file(filename.c_str());
  filename = pics_path+"numbers.bmp";
  printf ("loading: %s\n", filename.c_str());
  m_refPixbuf_Numbers    = Gdk::Pixbuf::create_from_file(filename.c_str());
  filename = pics_path+"batt_blocks.bmp";
  printf ("loading: %s\n", filename.c_str());
  m_refPixbuf_Blocks     = Gdk::Pixbuf::create_from_file(filename.c_str());

  m_back_width = m_refPixbuf_Background->get_width();
  m_back_height = m_refPixbuf_Background->get_height();
}

/* Expose callback for the drawing area */
bool GraphicsManager::on_drawingarea_expose(GdkEventExpose *event)
{
  gint rowstride = m_refPixbuf->get_rowstride();

  const guchar* pixels = m_refPixbuf->get_pixels() + (rowstride * event->area.y) + (event->area.x * 3);

  Glib::RefPtr<Gdk::Window> refWindow = m_DrawingArea.get_window();
  Glib::RefPtr<Gdk::GC> refGC = m_DrawingArea.get_style()->get_black_gc();

  refWindow->draw_rgb_image_dithalign(refGC,
                event->area.x, event->area.y,
                event->area.width, event->area.height,
                Gdk::RGB_DITHER_NORMAL,
                pixels, rowstride,
                event->area.x, event->area.y);

  return true;
}


void GraphicsManager::update_graphics(double voltage, double current, double charge)
{
  m_refPixbuf_Background->copy_area( 0, 0, m_back_width, m_back_height, m_refPixbuf, 0, 0);
 
  //voltage = 24.2;
  //current = 20.8;
  //charge = 100;
  char buff[10];
  sprintf(buff,"%4.1f",voltage);
  int len = strlen(buff);
  int point_off=0;
  for (int i=0;i<len;i++)
  {
      int off;
      int xpos;
      int ypos;
      GdkRectangle dest;
      if (buff[i]=='.')
          point_off=17;

      if (buff[i]>='0' && buff[i]<='9')
      {
        off=(buff[i]-'0')*29+point_off;    
        dest.x=19+i*29-point_off;
        dest.y=21;
        dest.width=29;
        dest.height=52;
        xpos=19+i*29-off;
        ypos=21;
      
        m_refPixbuf_Numbers->composite(m_refPixbuf,
                                    dest.x, dest.y,
                                    dest.width, dest.height,
                                    xpos, ypos,
                                    1, 1, Gdk::INTERP_NEAREST,255);
      }
  }

  sprintf(buff,"%4.1f",current);
  len = strlen(buff);
  point_off=0;
  for (int i=0;i<len;i++)
  {
      int off;
      int xpos;
      int ypos;
      GdkRectangle dest;
      if (buff[i]=='.')
          point_off=17;

      if (buff[i]>='0' && buff[i]<='9')
      {
        off=(buff[i]-'0')*29+point_off;    
        dest.x=19+i*29-point_off;
        dest.y=88;
        dest.width=29;
        dest.height=52;
        xpos=19+i*29-off;
        ypos=88;
      
        m_refPixbuf_Numbers->composite(m_refPixbuf,
                                    dest.x, dest.y,
                                    dest.width, dest.height,
                                    xpos, ypos,
                                    1, 1, Gdk::INTERP_NEAREST,255);
      }
  }

  int n_blocks = int (charge*11 / 100.0);
  for (int i=0;i<n_blocks;i++)
  {
      int off;
      int xpos;
      int ypos;
      GdkRectangle dest;

      off=0;    
      dest.x=166;
      dest.y=135-i*6;
      dest.width=14;
      dest.height=7;
      xpos=166;
      ypos=135-i*6-off;
      
      m_refPixbuf_Blocks->composite(m_refPixbuf,
                                    dest.x, dest.y,
                                    dest.width, dest.height,
                                    xpos, ypos,
                                    1, 1, Gdk::INTERP_NEAREST,255);
  }

  m_DrawingArea.queue_draw();
  m_frame_num++;
}
