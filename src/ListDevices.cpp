#include "utility.h"

int main(int argc, char *argv[])
{

    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(&argc, &argv);

    GstElement *source = gst_element_factory_make("tcambin", "source");
    GSList *serials = tcam_prop_get_device_serials(TCAM_PROP(source));

    printf("\n");
    for (GSList *elem = serials; elem; elem = elem->next)
    {
        const char *device_serial = (gchar *)elem->data;
        char *name;
        char *identifier;
        char *connection_type;
        gboolean ret = tcam_prop_get_device_info(TCAM_PROP(source),
                                                 device_serial,
                                                 &name,
                                                 &identifier,
                                                 &connection_type);
        if (ret)
        {
            printf("-> Model: %s Serial: %s Identifier: %s Type: %s\n",
                   name, device_serial, identifier, connection_type);
            g_free(name);
            g_free(identifier);
            g_free(connection_type);
        }
    }
    printf("\n");
    g_slist_free_full(serials, g_free);
    gst_object_unref(source);
    return 0;
}