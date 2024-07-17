/**
    @author Kenta Suzuki
*/

#ifndef rqt_gen3__my_plugin_H
#define rqt_gen3__my_plugin_H

#include <rqt_gui_cpp/plugin.h>

namespace rqt_gen3 {

class MyPlugin : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    MyPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
    virtual void shutdownPlugin() override;
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

    // Comment in to signal that the plugin has a way to configure it
    // virtual bool hasConfiguration() const override;
    // virtual void triggerConfiguration() override;
};

} // namespace

#endif // rqt_gen3__my_plugin_H
