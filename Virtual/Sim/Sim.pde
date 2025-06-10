import processing.serial.*;

Serial myPort;
float[] thetas = new float[5];
float[] linkLen = {0, 145, 220, 30, 0};
float axisLen = 60;

ArrayList<Float>[] posHist = (ArrayList<Float>[])new ArrayList[5];
ArrayList<Float>[] velHist = (ArrayList<Float>[])new ArrayList[5];
ArrayList<Float>[] accHist = (ArrayList<Float>[])new ArrayList[5];
float[] prevAngles = new float[5];
float[] prevVels = new float[5];

int panelW, graphW, panelH;
float dtSec;
color[] jointColors = {
  color(0, 0, 255),    // Joint1
  color(255, 165, 0),  // Joint2
  color(0, 128, 0),    // Joint3
  color(255, 0, 0),    // Joint4
  color(128, 0, 128)   // Joint5
};

void setup() {
  size(800, 600);
  frameRate(60);
  dtSec = 1.0 / 60.0;

  panelW = int(width * 0.6);   // 그래프 영역(오른쪽) 40% 확보
  graphW = width - panelW;
  panelH = height / 2;

  textSize(12);
  for (int i = 0; i < 5; i++) {
    posHist[i] = new ArrayList<Float>();
    velHist[i] = new ArrayList<Float>();
    accHist[i] = new ArrayList<Float>();
    prevAngles[i] = 0;
    prevVels[i] = 0;
  }

  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(255);
  updateHistories();

  drawDOF1Panel(0, 0, panelW/2, panelH);
  drawDOF5Panel(panelW/2, 0, panelW/2, panelH);
  drawDOF2to4Panel(0, panelH, panelW, panelH);

  int gx = panelW;
  int gh = height / 3;
  drawMultiGraph(gx, 0, graphW, gh, posHist, "Position (°)");
  drawMultiGraph(gx, gh, graphW, gh, velHist, "Velocity (°/s)");
  drawMultiGraph(gx, 2*gh, graphW, gh, accHist, "Acceleration (°/s²)");
}

void updateHistories() {
  for (int i = 0; i < 5; i++) {
    float cur = thetas[i];
    float rawVel = (cur - prevAngles[i]) / dtSec;
    // 더 강한 스무딩 (0.1)
    float vel = lerp(prevVels[i], rawVel, 0.1);
    float rawAcc = (vel - prevVels[i]) / dtSec;
    float prevAcc = (accHist[i].size()>0 ? accHist[i].get(accHist[i].size()-1) : 0);
    float acc = lerp(prevAcc, rawAcc, 0.1);

    addHistory(posHist[i], cur);
    addHistory(velHist[i], vel);
    addHistory(accHist[i], acc);

    prevAngles[i] = cur;
    prevVels[i] = vel;
  }
}

void addHistory(ArrayList<Float> list, float v) {
  list.add(v);
  // 히스토리 길이 4배로 확대
  if (list.size() > graphW*4) list.remove(0);
}

void drawMultiGraph(int x, int y, int w, int h, ArrayList<Float>[] data, String title) {
  noStroke(); fill(240);
  rect(x, y, w, 20);
  fill(0); textSize(12); textAlign(LEFT, TOP);
  text(title, x+5, y+2);

  for (int i = 0; i < 5; i++) {
    fill(jointColors[i]);
    text("J"+(i+1), x+5+i*30, y+h-15);
  }

  float minV = Float.MAX_VALUE, maxV = -Float.MAX_VALUE;
  for (int i = 0; i < 5; i++) for (float v : data[i]) { minV = min(minV,v); maxV = max(maxV,v); }
  if (minV == maxV) { minV -= 1; maxV += 1; }

  noFill(); strokeWeight(1);
  for (int j = 0; j < 5; j++) {
    stroke(jointColors[j]);
    beginShape();
    int len = data[j].size();
    int start = max(0, len - graphW);
    for (int i = start; i < len; i++) {
      float vx = map(i - start, 0, graphW, x, x + w);
      float vy = map(data[j].get(i), minV, maxV, y + h, y + 20);
      vertex(vx, vy);
    }
    endShape();
  }
}

void drawDOF1Panel(int x, int y, int w, int h) {
  noStroke(); fill(240); rect(x,y,w,20);
  fill(0); textSize(14); textAlign(LEFT, TOP);
  text("DOF1 (Roll)", x+5, y+2);

  float projLen = computeProjLen();
  float ang = radians(thetas[0]);
  float cx = x+w/2, cy = y+h/2;

  stroke(0,0,255); strokeWeight(4);
  line(cx, cy, cx + cos(ang)*projLen, cy + sin(ang)*projLen);
  PVector tip = new PVector(cx + cos(ang)*projLen, cy + sin(ang)*projLen);
  float ha1 = ang + radians(150), ha2 = ang - radians(150);
  line(tip.x, tip.y, tip.x + cos(ha1)*10, tip.y + sin(ha1)*10);
  line(tip.x, tip.y, tip.x + cos(ha2)*10, tip.y + sin(ha2)*10);

  noStroke(); fill(0); textSize(12);
  text(nf(thetas[0],1,2)+"°", tip.x+5, tip.y+5);
}

float computeProjLen() {
  float sum = 0;
  for (int i = 1; i <= 3; i++) sum += linkLen[i]*cos(radians(thetas[i]));
  return sum;
}

void drawDOF2to4Panel(int x, int y, int w, int h) {
  noStroke(); fill(240); rect(x,y,w,20);
  fill(0); textSize(14); textAlign(LEFT, TOP);
  text("DOF2-4 (Pitch-Pitch-Pitch)", x+5, y+2);

  float cx = x+w/2, cy = y+h/2;
  float sumAng = 0, px = cx, py = cy;
  stroke(0,0,255); strokeWeight(4);
  for (int i = 1; i <= 3; i++) {
    noStroke(); fill(0,0,255);
    ellipse(px, py, 10, 10);
    sumAng += radians(thetas[i]);
    float nx = px + linkLen[i]*cos(sumAng);
    float ny = py + linkLen[i]*sin(sumAng);
    stroke(0,0,255); noFill(); line(px, py, nx, ny);
    px = nx; py = ny;
  }
  noStroke(); fill(0,0,255); ellipse(px, py, 10, 10);
  // Gripper
  pushMatrix(); translate(px, py); rotate(sumAng);
  stroke(0); strokeWeight(4); noFill();
  line(0, -15, 70, -15);
  line(0,  15, 70,  15);
  line(0, -15, 0,   15);
  popMatrix();

  fill(0); textSize(12); textAlign(LEFT, TOP);
  sumAng = 0; px = cx; py = cy;
  for (int i = 1; i <= 3; i++) {
    text(nf(thetas[i],1,2)+"°", px+5, py-15);
    sumAng += radians(thetas[i]);
    px += linkLen[i]*cos(sumAng);
    py += linkLen[i]*sin(sumAng);
  }
}

void drawDOF5Panel(int x, int y, int w, int h) {
  noStroke(); fill(240); rect(x,y,w,20);
  fill(0); textSize(14); textAlign(LEFT, TOP);
  text("DOF5 (Roll)", x+5, y+2);

  float cx = x+w/2, cy = y+h/2;
  float ang = radians(thetas[4]);
  pushMatrix(); translate(cx, cy); rotate(ang);
  strokeWeight(4);
  stroke(255,0,0); line(0,0,axisLen,0);
  line(axisLen,0,axisLen-10,-5); line(axisLen,0,axisLen-10,5);
  stroke(0,255,0); line(0,0,0,-axisLen);
  line(0,-axisLen,-5,-axisLen+10); line(0,-axisLen,5,-axisLen+10);
  popMatrix();

  noStroke(); fill(0); textSize(12);
  text(nf(thetas[4],1,2)+"°", x+w-50, y+25);
}

void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  if (line == null) return;
  String[] v = split(trim(line), ',');
  if (v.length != 5) return;
  for (int i = 0; i < 5; i++) {
    try { thetas[i] = float(v[i]); } catch(Exception e) {}
  }
}
