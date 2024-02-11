import processing.core.*;
import processing.net.*;

PVector cameraPos;
PVector lookAt;
float cameraDistance;
float cameraSpeed = 2.0; // Adjust camera movement speed
float zoomSpeed = 0.5;
int floorSize = 1000; // in centimeters
float ballRadius = 10; // in centimeters
float ballHeight = 18; // in centimeters



void setup() {
  size(800, 600, P3D);
  cameraPos = new PVector(0, 0, 500); // start the camera a few meters away from the origin
  lookAt = new PVector(0, 0, 0); // look at the origin
  cameraDistance = PVector.dist(cameraPos, lookAt);


}

void draw() {
  background(255);

  // Set camera position and orientation
  camera(cameraPos.x, cameraPos.y, cameraPos.z, lookAt.x, lookAt.y, lookAt.z, 0, 1, 0);

  // Draw floor
  drawFloor();

  getRay(new PVector(-0.46105761, -0.36466165,  0.80897946));

  getRay(new PVector(-0.46142761, -0.3645868,   0.80880222));
  // Draw ball
  drawBall();

}

void getRay(PVector direction){


    // origin of the ray
    PVector origin = new PVector(0, 0, 18);

    float length = 100;

    PVector end = origin.copy().add(direction.copy().mult(length));
    fill(255, 0, 0);
    stroke(2);
    line(origin.x, origin.y, origin.z, end.x, end.y, end.z);

    // Draw the origin
    fill(0, 0, 255);
    noStroke();
    translate(origin.x, origin.y, origin.z);
    sphere(5);
    translate(-origin.x, -origin.y, -origin.z);

    // Draw the end
    fill(0, 255, 0);
    noStroke();
    translate(end.x, end.y, end.z);
    sphere(5);
    translate(-end.x, -end.y, -end.z);


}

// checkered floor of 1m squares
void drawFloor() {
  int numSquares = floorSize / 100;
  for (int i = -numSquares; i < numSquares; i++) {
    for (int j = -numSquares; j < numSquares; j++) {
      if ((i + j) % 2 == 0) {
        fill(255);
      } else {
        fill(0);
      }
      noStroke();
      rect(i * 100, j * 100, 100, 100);
    }
  }
}

void drawBall() {
  fill(255, 0, 0);
  noStroke();
  translate(0, 0, ballHeight);
  sphere(ballRadius);
  translate(0, 0, -ballHeight);
}

void keyPressed() {
  if (keyCode == UP) {
    PVector forward = PVector.sub(lookAt, cameraPos);
    forward.normalize();
    forward.mult(cameraSpeed);
    cameraPos.add(forward);
    lookAt.add(forward);
  } else if (keyCode == DOWN) {
    PVector backward = PVector.sub(cameraPos, lookAt);
    backward.normalize();
    backward.mult(cameraSpeed);
    cameraPos.add(backward);
    lookAt.add(backward);
  } else if (keyCode == LEFT) {
    PVector right = lookAt.cross(cameraPos);
    right.normalize();
    right.mult(cameraSpeed);
    cameraPos.add(right);
    lookAt.add(right);
  } else if (keyCode == RIGHT) {
    PVector left = cameraPos.cross(lookAt);
    left.normalize();
    left.mult(cameraSpeed);
    cameraPos.add(left);
    lookAt.add(left);
  }
}

void mouseDragged() {
  float dx = mouseX - pmouseX;
  float dy = mouseY - pmouseY;

  // Rotate camera around origin
  PVector dir = PVector.sub(cameraPos, lookAt);
  float radius = dir.mag();
  float theta = atan2(dir.y, dir.x);
  float phi = atan2(sqrt(dir.x * dir.x + dir.y * dir.y), dir.z);
  theta += dx * 0.01;
  phi -= dy * 0.01;
  phi = constrain(phi, 0.01, PI - 0.01);
  cameraPos.x = lookAt.x + radius * sin(phi) * cos(theta);
  cameraPos.y = lookAt.y + radius * sin(phi) * sin(theta);
  cameraPos.z = lookAt.z + radius * cos(phi);
}
