
#include "simproxml.h"

#include <stdio.h>

int main()
{
    printf("ok\n");

    simproxml::XMLDocument xml_document;

    const char *cfg_file_path = "../../../config/ComponentMngrCfg.xml";

    /* 加载配置文件 */
    const simproxml::XMLError loadResult = xml_document.LoadFile(cfg_file_path);
    if (loadResult != simproxml::XML_SUCCESS)
    {
        printf("open cfg file %s err.\n", cfg_file_path);
        printf("loadResult = %d\n", loadResult);
        return 0;
    }

    simproxml::XMLElement *root = xml_document.RootElement();
    if (root == NULL)
    {
        printf("root = NULL");
        return 0;
    }

    for (simproxml::XMLElement *level2Node = root->FirstChildElement(); level2Node != NULL; level2Node = level2Node->NextSiblingElement())
    {
        if (0 == strncmp(level2Node->Name(), "DynamicsPlugin", strlen("DynamicsPlugin")))
        {
            const char *name = level2Node->Attribute("name");
            if (NULL != name)
            {
                printf("name = %s\n", name);
            }

            for (simproxml::XMLElement *level3Node = level2Node->FirstChildElement(); level3Node != NULL; level3Node = level3Node->NextSiblingElement())
            {
                if (0 == strncmp(level3Node->Name(), "Load", strlen("Load")))
                {
                    const char *lib = level3Node->Attribute("lib");
                    if (NULL != lib)
                    {
                        printf("lib = %s\n", lib);
                    }
                }
            }
        }

        if (0 == strncmp(level2Node->Name(), "Sensor", strlen("Sensor")))
        {
            const char *name = level2Node->Attribute("name");
            if (NULL != name)
            {
                printf("name = %s\n", name);
            }

            const char *type = level2Node->Attribute("type");
            if (NULL != type)
            {
                printf("type = %s\n", type);
            }

            for (simproxml::XMLElement *level3Node = level2Node->FirstChildElement(); level3Node != NULL; level3Node = level3Node->NextSiblingElement())
            {
                if (0 == strncmp(level3Node->Name(), "Load", strlen("Load")))
                {
                    const char *lib = level3Node->Attribute("lib");
                    if (NULL != lib)
                    {
                        printf("lib = %s\n", lib);
                    }

                    const char *path = level3Node->Attribute("path");
                    if (NULL != path)
                    {
                        printf("path = %s\n", path);
                    }
                }
            }
        }
    }

    printf("load vissim cfg: %s ok.\n", cfg_file_path);


    return 0;
}

