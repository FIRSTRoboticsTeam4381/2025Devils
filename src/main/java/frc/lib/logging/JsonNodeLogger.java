// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import java.util.Map.Entry;

import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/** Add your docs here. */
@CustomLoggerFor(JsonNode.class)
public class JsonNodeLogger extends ClassSpecificLogger<JsonNode> {

    public JsonNodeLogger()
    {
        super(JsonNode.class);
    }

    @Override
    protected void update(EpilogueBackend backend, JsonNode object) {
        if(object != null) {
            // Recursively publish the node and its children

            if(object.isObject())
            {
                // Recursively read this object's items
                object.fields().forEachRemaining((Entry<String,JsonNode> e) -> {
                    
                    if(e.getValue().isValueNode())
                    {
                        logValueNode(backend, e.getKey(), e.getValue());   
                    }
                    else if(e.getValue().isContainerNode())
                    {
                        // Recursion time!
                        update(backend.getNested(e.getKey()), e.getValue());
                    }
                });

            }
            else if(object.isArray())
            {
                // Recursively read this array
                // Note: Not sure how to use Epilogue to make a "native" 
                // network tables array of arbitrary stuff

                // Not sure how to implement when we recurse over value nodes
                int i = 0;
                for(JsonNode n : object)
                {
                    update(backend.getNested(Integer.toString(i)), n);
                    i++;
                }


            }
            else if(object.isValueNode())
            {
                // How do we get the key for a value node?
                // We shouldn't end up here
                // because we have no idea what the key is
                // This only makes sense when we have an array I guess
                logValueNode(backend, "value", object);
            }
        }
    }

    
    private void logValueNode(EpilogueBackend backend, String key, JsonNode n)
    {
        // This is a key/value pair, log now
        JsonToken t = n.asToken();

        // Need to cast down to an appropriate logging type
        if(t == JsonToken.VALUE_NUMBER_FLOAT)
            backend.log(key, n.asDouble());
        else if(t == JsonToken.VALUE_NUMBER_INT)
            backend.log(key, n.asDouble());
        else if(t == JsonToken.VALUE_TRUE || t == JsonToken.VALUE_FALSE)
            backend.log(key, n.asBoolean());
        else
            backend.log(key, n.asText());

    }

    
}
