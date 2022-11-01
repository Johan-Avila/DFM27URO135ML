#include "utility.h"

void print_formats(GstElement *camera)
{

    GstPad *pad = gst_element_get_static_pad(camera, "src");
    GstCaps *caps = gst_pad_query_caps(pad, NULL);

    for (unsigned int i = 0; i < gst_caps_get_size(caps); ++i)
    {
        GstStructure *structure = gst_caps_get_structure(caps, i);
        const char *name = gst_structure_get_name(structure);
        GstCapsFeatures *features = gst_caps_get_features(caps, i);

        if (features)
        {
            if (gst_caps_features_contains(features, "memory:NVMM"))
            {
                printf("NVMM ");
            }
        }

        if (gst_structure_get_field_type(structure, "format") == G_TYPE_STRING)
        {
            const char *format = gst_structure_get_string(structure, "format");
            printf("%s %s - ", name, format);
        }
        else if (gst_structure_get_field_type(structure, "format") == GST_TYPE_LIST)
        {
            printf("%s { ", name);

            const GValue *val = gst_structure_get_value(structure, "format");

            for (unsigned int x = 0; x < gst_value_list_get_size(val); ++x)
            {
                const GValue *format = gst_value_list_get_value(val, x);
                printf("%s ", g_value_get_string(format));
            }
            printf("} - ");
        }
        else
        {
            printf("format handling not implemented for unexpected type: %s\n",
                   G_VALUE_TYPE_NAME(gst_structure_get_field_type(structure,
                                                                  "format")));
            continue;
        }

        GType width_type = gst_structure_get_field_type(structure, "width");

        if (width_type == GST_TYPE_INT_RANGE)
        {
            int width_min = gst_value_get_int_range_min(gst_structure_get_value(structure,
                                                                                "width"));
            int width_max = gst_value_get_int_range_max(gst_structure_get_value(structure,
                                                                                "width"));
            printf("width: [%d-%d]", width_min, width_max);
        }
        else
        {
            int width;
            gboolean ret = gst_structure_get_int(structure, "width", &width);
            if (!ret)
            {
                printf("Unable to query width\n");
                continue;
            }
            printf("%d", width);
        }

        printf(" X ");

        GType height_type = gst_structure_get_field_type(structure, "height");

        if (height_type == GST_TYPE_INT_RANGE)
        {
            int height_min = gst_value_get_int_range_min(gst_structure_get_value(structure,
                                                                                 "height"));
            int height_max = gst_value_get_int_range_max(gst_structure_get_value(structure,
                                                                                 "height"));
            printf("height: [%d-%d]", height_min, height_max);
        }
        else
        {
            int height;
            gboolean ret = gst_structure_get_int(structure, "height", &height);
            if (!ret)
            {
                printf("Unable to query height\n");
                continue;
            }
            printf("%d", height);
        }

        printf(" - ");

        const GValue *framerate = gst_structure_get_value(structure, "framerate");

        if (G_VALUE_TYPE(framerate) == GST_TYPE_LIST)
        {
            for (unsigned int x = 0; x < gst_value_list_get_size(framerate); ++x)
            {
                const GValue *val = gst_value_list_get_value(framerate, x);
                if (G_VALUE_TYPE(val) == GST_TYPE_FRACTION)
                {
                    int num = gst_value_get_fraction_numerator(val);
                    int den = gst_value_get_fraction_denominator(val);

                    printf("%d/%d ", num, den);
                }
                else
                {
                    printf("Handling of framerate handling not implemented for non fraction types.\n");
                    break;
                }
            }
        }
        else if (G_VALUE_TYPE(framerate) == GST_TYPE_FRACTION_RANGE)
        {
            const GValue *framerate_min = gst_value_get_fraction_range_min(framerate);
            const GValue *framerate_max = gst_value_get_fraction_range_max(framerate);
            printf("%d/%d - %d/%d",
                   gst_value_get_fraction_numerator(framerate_min),
                   gst_value_get_fraction_denominator(framerate_min),
                   gst_value_get_fraction_numerator(framerate_max),
                   gst_value_get_fraction_denominator(framerate_max));
        }
        else
        {
            printf("Unable to interpret framerate type\n");
            continue;
        }
        printf("\n");
    }
    gst_caps_unref(caps);
    gst_object_unref(pad);
}

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
            printf("\n");
            g_free(name);
            g_free(identifier);
            g_free(connection_type);

            string pip_des;
            pip_des.append("tcambin name=camera serial=");
            pip_des.append(device_serial);
            pip_des.append(" ! fakesink");

            GError *err = NULL;
            const char *pipeline_description = pip_des.c_str();
            GstElement *pipeline = gst_parse_launch(pipeline_description, &err);
            if (pipeline == NULL)
            {
                printf("Unable to create pipeline: %s\n", err->message);
                g_free(err);
                return 1;
            }
            GstElement *camera = gst_bin_get_by_name(GST_BIN(pipeline), "camera");

            gst_element_set_state(camera, GST_STATE_READY);

            print_formats(camera);
            printf("\n");

            gst_element_set_state(camera, GST_STATE_NULL);

            gst_object_unref(camera);
            gst_object_unref(pipeline);
        }
    }
    g_slist_free_full(serials, g_free);
    gst_object_unref(source);
    return 0;
}