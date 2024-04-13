/**
    @author Kenta Suzuki
*/

#include "rqt_gen3_lite/my_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include "rqt_gen3_lite/my_widget.h"

namespace rqt_gen3_lite {

MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    auto widget_ = new MyWidget;
    if(context.serialNumber() > 1) {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    // add widget to the user interface
    context.addWidget(widget_);
}

void MyPlugin::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
    (void) plugin_settings;
    (void) instance_settings;
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
    (void) plugin_settings;
    (void) instance_settings;
}

/*bool MyPlugin::hasConfiguration() const
{
    //   return true;
}

void MyPlugin::triggerConfiguration()
{
    // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_gen3_lite::MyPlugin, rqt_gui_cpp::Plugin)
