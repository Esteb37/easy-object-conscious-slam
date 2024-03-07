import processing.net.*;

Client client;

void setup()
{
  size(1000, 1000);
  client = new Client(this, "localhost", 12345);
}

// matrix of size 100000x100000 points
PVector[][] matrix = new PVector[1000][1000];

void draw()
{
  JSONObject json = null;

  // Read data from the socket
  if (client.available() > 0)
  {
    String data = client.readString();
    // split by newlines
    // parse the first line as json
    json = parseJSONObject(data);
  }

  if(json != null){

    background(255);

    JSONObject lidar = json.getJSONObject("lidar");
    JSONObject pose = json.getJSONObject("pose");

    JSONArray ranges = lidar.getJSONArray("ranges");




    for (int i = 0; i < ranges.size(); i++)
    {
      try
      {
        float angle = lidar.getFloat("angle_min") + i * lidar.getFloat("angle_increment");
        float range = ranges.getFloat(i);

        float x = range * cos(angle);
        float y = range * sin(angle);


        x = x * 50 + width / 2;
        y = y * 50 + height / 2;

        println(floor(x),floor(y));

        fill(255, 0, 0);
        ellipse(x, y, 1, 1);
        ellipse(width/2, height/2, 10, 10);




      }
      catch (Exception e)
      {
        // NaNs and infs
      }
    }
  }
}
