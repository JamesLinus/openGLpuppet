#include "appwindow.hpp"

AppWindow::AppWindow(SceneNode *root) : m_viewer(root)
{
  set_title("Advanced Ergonomics Laboratory");

  // A utility class for constructing things that go into menus, which
  // we'll set up next.
  using Gtk::Menu_Helpers::MenuElem;
  using Gtk::Menu_Helpers::RadioMenuElem;
  using Gtk::Menu_Helpers::CheckMenuElem;
  sigc::slot1<void, Viewer::Mode> mode_slot = sigc::mem_fun(m_viewer, &Viewer::set_mode);
  
  // Set up the application menu
  // The slot we use here just causes AppWindow::hide() on this,
  // which shuts down the application.
  m_menu_app.items().push_back(MenuElem("_Quit", Gtk::AccelKey("q"),
    sigc::mem_fun(*this, &AppWindow::hide)));
  m_menu_app.items().push_back(MenuElem("Reset Pos_ition", Gtk::AccelKey("i"),
    sigc::mem_fun(m_viewer, &Viewer::reset_position)));
  m_menu_app.items().push_back(MenuElem("Reset _Orientation", Gtk::AccelKey("o"),
    sigc::mem_fun(m_viewer, &Viewer::reset_orientation)));
  m_menu_app.items().push_back(MenuElem("Reset Joi_nts", Gtk::AccelKey("n"),
    sigc::mem_fun(m_viewer, &Viewer::reset_joints)));
  m_menu_app.items().push_back(MenuElem("Reset _All", Gtk::AccelKey("a"),
    sigc::mem_fun(m_viewer, &Viewer::reset_all)));

  m_menu_mode.items().push_back(RadioMenuElem(mode_group, "_Position and Orientation", Gtk::AccelKey("p"),
    sigc::bind(mode_slot, Viewer::TRACK)));
  m_menu_mode.items().push_back(RadioMenuElem(mode_group, "_Joints", Gtk::AccelKey("j"),
    sigc::bind(mode_slot, Viewer::JOINT)));

  m_menu_edit.items().push_back(MenuElem("_Undo", Gtk::AccelKey("u"),
    sigc::mem_fun(m_viewer, &Viewer::undo)));
  m_menu_edit.items().push_back(MenuElem("_Redo", Gtk::AccelKey("r"),
    sigc::mem_fun(m_viewer, &Viewer::redo)));
  
  m_menu_opt.items().push_back(CheckMenuElem("_Circle", Gtk::AccelKey("c"),
    sigc::mem_fun(m_viewer, &Viewer::toggle_circle)));
  m_menu_opt.items().push_back(CheckMenuElem("_Z-buffer", Gtk::AccelKey("z"),
    sigc::mem_fun(m_viewer, &Viewer::depth_buffer)));
  m_menu_opt.items().push_back(CheckMenuElem("_Backface cull", Gtk::AccelKey("b"),
    sigc::mem_fun(m_viewer, &Viewer::back_culling)));
  m_menu_opt.items().push_back(CheckMenuElem("_Frontface cull", Gtk::AccelKey("f"),
    sigc::mem_fun(m_viewer, &Viewer::front_culling)));

  // Set up the menu bar
  m_menubar.items().push_back(Gtk::Menu_Helpers::MenuElem("_Application", m_menu_app));
  m_menubar.items().push_back(Gtk::Menu_Helpers::MenuElem("_Mode", m_menu_mode));
  m_menubar.items().push_back(Gtk::Menu_Helpers::MenuElem("_Edit", m_menu_edit));
  m_menubar.items().push_back(Gtk::Menu_Helpers::MenuElem("_Options", m_menu_opt));
  
  // Pack in our widgets
  
  // First add the vertical box as our single "top" widget
  add(m_vbox);

  // Put the menubar on the top, and make it as small as possible
  m_vbox.pack_start(m_menubar, Gtk::PACK_SHRINK);

  // Put the viewer below the menubar. pack_start "grows" the widget
  // by default, so it'll take up the rest of the window.
  m_viewer.set_size_request(300, 300);
  m_vbox.pack_start(m_viewer);

  show_all();
}
