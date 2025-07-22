// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotController;

/** Class to poll the /status endpoint of a radio and put the results on NetworkTables.
 * This class assumes NetworkTables is already being auto-logged.
 */
public class RadioLogger {

    // Target radio IP
    private String target;

    // Timing intervals for querying status in microseconds
    // This does not include the time taken for the request to arrive
    // Radios refresh data roughly every 5 seconds
    private long pollRate = 250000*4*5;

    // Timeout length for when the endpoint doesn't respond, in milliseconds
    private long timeout = 2000;

    @Logged
    short resNum = 0;


    private HttpClient client = HttpClient.newHttpClient();
    private HttpRequest request;

    private long lastResultsTimestamp = RobotController.getFPGATime();
    private boolean requestInFlight = false;
    private CompletableFuture<String> liveRequest;

    private String latestResults = "";

    @Logged
    private String requestStatus = "Initializing...";

    private ObjectMapper mapper = new ObjectMapper();

    private JsonNode latestJson;

    public RadioLogger(String ip)
    {
        target = ip;

        request = HttpRequest.newBuilder()
         .uri(URI.create(target))
         .timeout(Duration.ofMillis(timeout))
         .header("Accept","application/json")
         .build();
    }


    private void query()
    {
        requestInFlight = true;
        liveRequest = client.sendAsync(request, BodyHandlers.ofString())
            .thenApply(HttpResponse::body);
        
        lastResultsTimestamp = RobotController.getFPGATime();
    }


    private void checkResult()
    {
        if(liveRequest.isDone())
        {
            requestInFlight = false;
            
            try {
                latestResults = liveRequest.get();
                requestStatus = "OK";
                resNum++;
            } catch (InterruptedException | ExecutionException | CancellationException e) {
                requestStatus = e.getLocalizedMessage();
            }

            try {
                latestJson = mapper.readTree(latestResults);
            } catch (JsonProcessingException e) {
                requestStatus = "Parse failure: "+e.getLocalizedMessage();
            }
        }
    }

   

    @Logged
    public JsonNode logData()
    {
        // If a query is in flight, check if it is done
        if(requestInFlight)
        {
            checkResult();
        }
        // Check if we need to send another request
        else if(!requestInFlight && RobotController.getFPGATime() > lastResultsTimestamp+pollRate)
        {
            query();
        }

        return latestJson;
    }

   
    
}

