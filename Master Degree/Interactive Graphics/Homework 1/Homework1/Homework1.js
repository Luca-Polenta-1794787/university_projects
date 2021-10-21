"use strict";

var shadedCube = function() {

var canvas;
var gl;

var numPositions = 186;

var positionsArray = [];
var normalsArray = [];

var vertices = [
   vec4(-0.05, -0.03,  0.03, 1.0),  // From here the central parallelepiped
   vec4(-0.05,  0.03,  0.03, 1.0),
   vec4(0.05,  0.03,  0.03, 1.0),
   vec4(0.05, -0.03,  0.03, 1.0),
   vec4(-0.05, -0.03, -0.03, 1.0),
   vec4(-0.05,  0.03, -0.03, 1.0),
   vec4(0.05,  0.03, -0.03, 1.0),
   vec4(0.05, -0.03, -0.03, 1.0),
   vec4(-0.08, 0.0, 0.0, 1.0),      // Tip of the left pyramid with a rectangular base
   vec4(0.08, 0.0, 0.0, 1.0),       // Tip of the right spyramid with a rectangular base
   vec4(-0.03, -0.07, 0.015, 1.0),  // From here the bottom trapezoid
   vec4(0.03, -0.07, 0.015, 1.0),
   vec4(-0.03, -0.07, -0.015, 1.0),
   vec4(0.03, -0.07, -0.015, 1.0),
   vec4(-0.03, 0.07, 0.015, 1.0),   // From here the top trapezoid
   vec4(0.03, 0.07, 0.015, 1.0),
   vec4(-0.03, 0.07, -0.015, 1.0),
   vec4(0.03, 0.07, -0.015, 1.0),
   vec4(-0.03, -0.015, 0.07, 1.0),  // From here the front trapezoid
   vec4(0.03, -0.015, 0.07, 1.0),
   vec4(-0.03, 0.015, 0.07, 1.0),
   vec4(0.03, 0.015, 0.07, 1.0),
   vec4(0.03, -0.015, -0.07, 1.0),  // From here the back trapezoid
   vec4(-0.03, -0.015, -0.07, 1.0),
   vec4(-0.03, 0.015, -0.07, 1.0),
   vec4(0.03, 0.015, -0.07, 1.0),
   vec4(0.0, 0.15, 0.0, 1.0),       // Tip of the top pyramid with a rectangular base
];

var ctm;
var viewerPos;
var program;
var program2;

/////////// TEXTURE ////////////
var texSize = 256;
var texCoordsArray = [];
var texture;
var texCoord = [
    vec2(0, 0),
    vec2(0, 1),
    vec2(1, 1),
    vec2(1, 0)
];
////////////////////////////////


///////// BARYCENTER //////////
var directionRotation = true;
var Trans = mat4(1, 0, 0, 0,
                 0, 1, 0, 0.02,
                 0, 0, 1, 0,
                 0, 0, 0, 1);
///////////////////////////////


///////// MODEL VIEW ///////////
const at = vec3(0.0, 0.0, 0.0);
const up = vec3(0.0, 1.0, 0.0);
var radius = 0.4;
var thetaEx3 = 0.0;
var phi = 1.0;
////////////////////////////////


///////// PERSPECTIVE ///////////
var near = 0.1;
var far = 4.0;
var dr = Math.PI/180.0;
var fovy = 60.0;  
var aspect;
var aspectForReset; 
var modelViewMatrixLoc, projectionMatrixLoc;
var modelViewMatrix, projectionMatrix;
////////////////////////////////


//////// BASIC MATERIAL ////////
var materialAmbient = vec4(0.3, 0.22, 0.0, 1.0);
var materialDiffuse = vec4(1.0, 0.775, 0.0, 1.0);
var materialSpecular = vec4(0.32, 0.32, 0.32, 1.0);
var materialShininess = 100.0;
var lightPosition = vec4(1.0, 1.0, 1.0, 0.0);
var lightAmbient = vec4(0.3, 0.3, 0.3, 1.0);
var lightDiffuse = vec4(0.9, 0.9, 0.9, 1.0);
var lightSpecular = vec4(1.0, 1.0, 1.0, 1.0);
///////////////////////////////


///// PER VERTEX/FRAGMENT /////
var PerVertex = true;
var nMatrix, nMatrixLoc;
///////////////////////////////


////////// CYLINDER ////////////
var ncylinder;
var flagNeonLight = false;
var modelViewMatrixLocCylinder, projectionMatrixLocCylinder;
var nMatrixCylinder, nMatrixLocCylinder;
var modelViewMatrixCylinder, projectionMatrixCylinder;
var lightPositionCyl1 = vec4(0.0, -0.15, 0.0, 0.0);
var lightPositionCyl2 = vec4(0.08, -0.15, 0.0, 0.0);
var lightPositionCyl3 = vec4(-0.08, -0.15, 0.0, 0.0);
var lightAmbientCyl = vec4(0.3, 0.2, 0.2, 1.0);
var lightDiffuseCyl = vec4(0.35, 0.0, 0.0, 1.0);
var lightSpecularCyl = vec4(1.0, 0.0, 0.0, 1.0);

var materialAmbientCyl =  vec4(1.0, 0.0, 0.0, 0.0);
var materialDiffuseCyl = vec4(0.4, 0.0, 0.0, 0.0);
var materialSpecularCyl = vec4(1.0, 0.0, 0.0, 0.0);
var emissiveCyl = vec4(0.2, 0.2, 0.2, 0.0);
var materialShininessCyl = 100.0;
//////////////////////////////// 


//////// ROUGH TEXTURE /////////
var tangentsArray = [];
var flagRoughTexture = false;
var flagTypeOfRoughTexture = false;

var data = new Array()
var howMuch = 1.0;
for (var i = 0; i<= texSize; i++)  data[i] = new Array();
for (var i = 0; i<= texSize; i++) for (var j=0; j<=texSize; j++)
   data[i][j] = 0.0;

//// GENERATION OF THE ROUGH SURFACE IN 3 NESTED LOOPS ////
for(var l=0; l<=60; l+=3){
   for(var k = 1; k<=60; k+=2){
      for(var i = k*texSize/64; i<(2+k)*texSize/64; i++){
         for (var j = (l+k)*texSize/64; j<(1+l+k)*texSize/64; j++){
            data[i][j] = howMuch;
         }
      }
   }
}
for(var l=0; l<=52; l+=4){
   for(var k = 1; k<=26-l; k+=2){
      for(var i =(4+k+l)*texSize/64; i<(6+k+l)*texSize/64; i++){
         for (var j = (1+k+(l/4))*texSize/64; j<(2+k+(l/4))*texSize/64; j++){
            data[i][j] = howMuch;
         }
      }
   }
}
for(var i =(7)*texSize/64; i<(9)*texSize/64; i++){
   for (var j = (1)*texSize/64; j<(2)*texSize/64; j++){
      data[i][j] = howMuch;
   }
   for (var j = (4)*texSize/64; j<(5)*texSize/64; j++){
      data[i][j] = howMuch;
   }
}
for(var k=0; k<26; k+=1){
   for(var i =(9+2*k)*texSize/64; i<(11+2*k)*texSize/64; i++){
      for(var l=0; l<55; l+=3){
         for (var j = (3+l-(k%3))*texSize/64; j<(4+l-(k%3))*texSize/64; j++){
            data[i][j] = howMuch;
         }
      }
   }
}

/* // NOT SUPER DENSE ROUGH TEXTURE

   for(var l=0; l<=30; l+=3){
      for(var k = 1; k<=30; k+=2){
         for(var i = k*texSize/32; i<(2+k)*texSize/32; i++){
            for (var j = (l+k)*texSize/32; j<(1+l+k)*texSize/32; j++){
               data[i][j] = howMuch;
            }
         }
      }
   }
   for(var l=0; l<=26; l+=4){
      for(var k = 1; k<=13-l; k+=2){
         for(var i =(4+k+l)*texSize/32; i<(6+k+l)*texSize/32; i++){
            for (var j = (1+k+(l/4))*texSize/32; j<(2+k+(l/4))*texSize/32; j++){
               data[i][j] = howMuch;
            }
         }
      }
   }
   for(var i =(7)*texSize/32; i<(9)*texSize/32; i++){
      for (var j = (1)*texSize/32; j<(2)*texSize/32; j++){
         data[i][j] = howMuch;
      }
      for (var j = (4)*texSize/32; j<(5)*texSize/32; j++){
         data[i][j] = howMuch;
      }
   }
   for(var k=0; k<11; k+=1){
      for(var i =(9+2*k)*texSize/32; i<(11+2*k)*texSize/32; i++){
         for(var l=0; l<25; l+=3){
            for (var j = (3+l-(k%3))*texSize/32; j<(4+l-(k%3))*texSize/32; j++){
               data[i][j] = howMuch;
            }
         }
      }
   }
*/

// Bump Map Normals
var normalst = new Array()
      for (var i=0; i<texSize; i++)  normalst[i] = new Array();
      for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
         normalst[i][j] = new Array();
      for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
         normalst[i][j][0] = data[i][j]-data[i+1][j];
         normalst[i][j][1] = data[i][j]-data[i][j+1];
         normalst[i][j][2] = 1;
      }

// Scale to Texture Coordinates
      for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
         var d = 0;
         for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
         d = Math.sqrt(d);
         for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
      }
   
var normalsBumpTexture = new Uint8Array(3*texSize*texSize);
// Normal Texture Array
      for ( var i = 0; i < texSize; i++ )
         for ( var j = 0; j < texSize; j++ )
            for(var k =0; k<3; k++)
               normalsBumpTexture[3*texSize*i+3*j+k] = 255*normalst[i][j][k];
////////////////////////////////



var xAxis = 0;
var yAxis = 1;
var zAxis = 2;
var axis = 0;
var thetaRot = vec3(0, 0, 0);

var thetaLoc;

var flag = false;

init();

////////// TEXTURE //////////
function configureTexture(passedImage) {
   texture = gl.createTexture();
   gl.bindTexture(gl.TEXTURE_2D, texture);
   gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
   gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, texSize, texSize, 0, gl.RGBA, gl.UNSIGNED_BYTE, passedImage);
   gl.generateMipmap(gl.TEXTURE_2D);
   gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
   gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
};
function configureTextureBumpMapping(passedImage) {
   var texture = gl.createTexture();
   gl.activeTexture(gl.TEXTURE0);
   gl.bindTexture(gl.TEXTURE_2D, texture);
   gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, passedImage);
   gl.generateMipmap(gl.TEXTURE_2D);
  }
/////////////////////////////

function quad(a, b, c, d) {
     var t1 = subtract(vertices[b], vertices[a]);
     var t2 = subtract(vertices[c], vertices[b]);
     var normal = cross(t1, t2);
     normal = vec3(normal);

     positionsArray.push(vertices[a]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[0]);
     tangentsArray.push(t1);
     positionsArray.push(vertices[b]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[1]);
     tangentsArray.push(t1);
     positionsArray.push(vertices[c]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[2]);
     tangentsArray.push(t1);
     positionsArray.push(vertices[a]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[0]);
     tangentsArray.push(t1);
     positionsArray.push(vertices[c]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[2]);
     tangentsArray.push(t1);
     positionsArray.push(vertices[d]);
     normalsArray.push(normal);
     texCoordsArray.push(texCoord[3]);
     tangentsArray.push(t1);
}

function colorMyObject()
{
    /* 
    // This is the central parallelepiped that is completely internal and I haven't drawn it. NOT COUNTED in numPosition
    //quad(1, 0, 3, 2);
    //quad(2, 3, 7, 6);
    //quad(3, 0, 4, 7);
    //quad(6, 5, 1, 2);
    //quad(4, 5, 6, 7);
    //quad(5, 4, 0, 1);
    */

    //TRIANGLE - LEFT
    quad(1, 8, 0, 0) 
    quad(0, 8, 4, 4) 
    quad(5, 8, 1, 1) 
    quad(4, 8, 5, 5)
    
    //TRIANGLE - RIGHT
    quad(3, 9, 2, 2) 
    quad(6, 9, 7, 7) 
    quad(7, 9, 3, 3) 
    quad(2, 9, 6, 6)

    // BOTTOM
    quad(0, 10, 11, 3);
    quad(3, 11, 13, 7);
    quad(13, 11, 10, 12);
    //quad(6, 5, 1, 2);     // Bottom: it is useless because it is internal and hidden. NOT COUNTED in numPosition
    quad(7, 13, 12, 4);
    quad(4, 12, 10, 0);
   
    // TOP
    quad(2, 15, 14, 1);
    quad(6, 17, 15, 2);
    //quad(2, 1, 5, 6);     // Bottom: it is useless because it is internal and hidden. NOT COUNTED in numPosition
    //quad(16, 14, 15, 17); // Top: it is useless because it is internal and hidden. NOT COUNTED in numPosition
    quad(5, 16, 17, 6);
    quad(1, 14, 16, 5);

    // FRONT
    quad(3, 19, 18, 0);
    quad(2, 21, 19, 3);
    //quad(3, 0, 1, 2);     // Bottom: it is useless because it is internal and hidden. NOT COUNTED in numPosition
    quad(19, 21, 20, 18);
    quad(1, 20, 21, 2);
    quad(0, 18, 20, 1);
    
    // BACK
    quad(4, 23, 22, 7);
    quad(7, 22, 25, 6);
    //quad(7, 4, 5, 6);     // Bottom: it is useless because it is internal and hidden. NOT COUNTED in numPosition
    quad(23, 24, 25, 22);
    quad(6, 25, 24, 5);
    quad(5, 24, 23, 4); 

    //TRIANGLE - TOP
    quad(15, 26, 14, 14) 
    quad(17, 26, 15, 15) 
    quad(16, 26, 17, 17) 
    quad(14, 26, 16, 16)
}

// Loading the image as texture
var image = new Image();
image.onload = function(){
   configureTexture(image);
}
image.src = "TIGER.png"

function init() {
    canvas = document.getElementById("gl-canvas");

    gl = canvas.getContext('webgl2');
    if (!gl) alert( "WebGL 2.0 isn't available");

    gl.viewport(0, 0, canvas.width, canvas.height);

    aspect = canvas.width/canvas.height;
    aspectForReset = canvas.width/canvas.height;

    gl.clearColor(1.0, 1.0, 1.0, 1.0);

    gl.enable(gl.DEPTH_TEST);

    //
    //  Load shaders and initialize attribute buffers
    //
    program = initShaders(gl, "vertex-shader", "fragment-shader");
    program2 = initShaders(gl, "vertex-shader-Cylinder", "fragment-shader-Cylinder");

    colorMyObject();
    
    ////////// CYLINDER ////////////
    var dataCylinder = cylinder(144, 6, true);
    dataCylinder.scale(0.1, 0.1, 0.1);
    dataCylinder.rotate(270.0, [0, 0, 1]);
    dataCylinder.rotate(90.0, [1, 0, 0]);
    dataCylinder.translate(0, -0.15, 0.1);
    ncylinder = dataCylinder.TriangleVertices.length;
    positionsArray = positionsArray.concat(dataCylinder.TriangleVertices);
    normalsArray = normalsArray.concat(dataCylinder.TriangleNormals);
    texCoordsArray = texCoordsArray.concat(dataCylinder.TextureCoordinates);
    tangentsArray = tangentsArray.concat(dataCylinder.TriangleTangents);
    ////////////////////////////////

    var nBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);
    
    var normalLoc = gl.getAttribLocation(program, "aNormal");
    gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normalLoc);
    
    var normal2Loc = gl.getAttribLocation(program2, "aNormal");
    gl.vertexAttribPointer(normal2Loc, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(normal2Loc);

    var vBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(positionsArray), gl.STATIC_DRAW);

    var positionLoc = gl.getAttribLocation(program, "aPosition");
    gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(positionLoc);

    var position2Loc = gl.getAttribLocation(program2, "aPosition");
    gl.vertexAttribPointer(position2Loc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(position2Loc); 

    /////////// BASIC TEXTURE ///////////
    var tBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);

    var texCoordLoc = gl.getAttribLocation(program, "aTexCoord");
    gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(texCoordLoc);

    configureTexture(image);
    /////////////////////////////////////

    /////////// ROUGH TEXTURE ///////////
    var tangBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, tangBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, flatten(tangentsArray), gl.STATIC_DRAW);

    var tangentLoc = gl.getAttribLocation(program, "aTangent");
    gl.vertexAttribPointer(tangentLoc, 4, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(tangentLoc);
    /////////////////////////////////////

    thetaLoc = gl.getUniformLocation(program, "theta");
    
    document.getElementById("ButtonX").onclick = function(){axis = xAxis;};
    document.getElementById("ButtonY").onclick = function(){axis = yAxis;};
    document.getElementById("ButtonZ").onclick = function(){axis = zAxis;};
    document.getElementById("ButtonDirection").onclick = function(){directionRotation = !directionRotation;};
    document.getElementById("ButtonT").onclick = function(){flag = !flag;};
    document.getElementById("zFarSlider").onchange = function(event){far = event.target.value; console.log("far = "+far); };
    document.getElementById("zNearSlider").onchange = function(event){near = event.target.value; console.log("near = "+near); };
    document.getElementById("radiusSlider").onchange = function(event){radius = event.target.value; console.log("radius = "+radius); };
    document.getElementById("thetaSlider").onchange = function(event){thetaEx3 = event.target.value*dr; console.log("thetaEx3 = "+thetaEx3); };
    document.getElementById("phiSlider").onchange = function(event){phi = event.target.value*dr; console.log("phi = "+phi); };
    document.getElementById("aspectSlider").onchange = function(event){aspect = event.target.value; console.log("aspect = "+aspect); };
    document.getElementById("fovySlider").onchange = function(event){fovy = event.target.value; console.log("fovy = "+fovy); };
    document.getElementById("ButtonPerVertexPerFragment").onclick = function(){
         PerVertex=!PerVertex;
         if(PerVertex==false){
            document.getElementById("ButtonPerVertexPerFragment").innerHTML="Activate Per-Vertex (Deactivating Per-Fragment)";
         }else{
            document.getElementById("ButtonPerVertexPerFragment").innerHTML="Activate Per-Fragment (Deactivating Per-Vertex)";
         }
         gl.useProgram(program2);
         gl.uniform1f(gl.getUniformLocation(program2, "uPerVertex"), PerVertex);
         gl.useProgram(program);
         gl.uniform1f(gl.getUniformLocation(program, "uPerVertex"), PerVertex);
         // Checking that the rough texture is computed only when there is per-fragment shading
         if(flagRoughTexture==true){
            flagRoughTexture=!flagRoughTexture;
            configureTexture(image);
            gl.uniform1i(gl.getUniformLocation(program, "uTextureMap"), 0);
            gl.uniform1f(gl.getUniformLocation(program, "uRoughTexture"), flagRoughTexture);
            document.getElementById("ButtonRoughTexture").innerHTML="Activate Rough Texture (Deactivating the Tiger Texture)";
            console.log("Deactivating the Rough Texture when the Per-Vertex is activated");
            fovy = 60.0;
            document.getElementById("fovySlider").value = fovy;
         }
         console.log("PerVertex = "+PerVertex);
    };
    document.getElementById("ButtonReset").onclick = function(){
         radius = 0.4; thetaEx3 = 0.0; phi = 1.0; near = 0.1; far = 4.0; fovy = 60.0; aspect = aspectForReset;
         document.getElementById("zFarSlider").value = far;       document.getElementById("zNearSlider").value = near;
         document.getElementById("radiusSlider").value = radius;  document.getElementById("thetaSlider").value = thetaEx3;
         document.getElementById("phiSlider").value = phi;        document.getElementById("aspectSlider").value = aspectForReset;
         document.getElementById("fovySlider").value = fovy;
    };
    document.getElementById("ButtonNeonLight").onclick = function(){
         flagNeonLight=!flagNeonLight;
         if(flagNeonLight==true){
            document.getElementById("ButtonNeonLight").innerHTML="Deactivate Neon Light";
         }else{
            document.getElementById("ButtonNeonLight").innerHTML="Activate Neon Light";
         }
         gl.useProgram(program2);
         gl.uniform1f(gl.getUniformLocation(program2, "uNeonLight"), flagNeonLight);
         gl.useProgram(program);
         gl.uniform1f(gl.getUniformLocation(program, "uNeonLight"), flagNeonLight);
         // Checking that the rough texture is computed only when the neon light is off
         if(flagRoughTexture==true){
            flagRoughTexture=!flagRoughTexture;
            configureTexture(image);
            gl.uniform1i(gl.getUniformLocation(program, "uTextureMap"), 0);
            gl.uniform1f(gl.getUniformLocation(program, "uRoughTexture"), flagRoughTexture);
            document.getElementById("ButtonRoughTexture").innerHTML="Activate Rough Texture (Deactivating the Tiger Texture)";
            console.log("Automatic Deactivating the Rough Texture when the Neon Light is on");
            fovy = 60.0;
            document.getElementById("fovySlider").value = fovy;
         }
         console.log("flagNeonLight = "+flagNeonLight);
    };
    document.getElementById("ButtonRoughTexture").onclick = function(){
         flagRoughTexture=!flagRoughTexture;
         gl.useProgram(program);
         if(!flagRoughTexture){
            gl.uniform1f(gl.getUniformLocation(program, "uRoughTexture"), flagRoughTexture);
            configureTexture(image);
            gl.uniform1i(gl.getUniformLocation(program, "uTextureMap"), 0);
            document.getElementById("ButtonRoughTexture").innerHTML="Activate Rough Texture (Deactivating the Tiger Texture)";
            fovy = 60.0;
            document.getElementById("fovySlider").value = fovy;
         }else{
            gl.uniform1f(gl.getUniformLocation(program, "uRoughTexture"), flagRoughTexture);
            configureTextureBumpMapping(normalsBumpTexture);
            gl.uniform1i(gl.getUniformLocation(program, "uTextureMap"), 0);
            document.getElementById("ButtonRoughTexture").innerHTML="Activate Tiger Texture (Deactivating the Rough Texture)";
            fovy = 30.0;
            document.getElementById("fovySlider").value = fovy;
            // Checking that the rough texture is computed only when there is per-fragment shading and the neon light is off
            if(flagNeonLight==true){
               flagNeonLight=!flagNeonLight;
               document.getElementById("ButtonNeonLight").innerHTML="Activate Neon Light";
               gl.uniform1f(gl.getUniformLocation(program, "uNeonLight"), flagNeonLight);
               gl.useProgram(program2);
               gl.uniform1f(gl.getUniformLocation(program2, "uNeonLight"), flagNeonLight);
               console.log("Automatic Deactivating of the Neon Light");
            }
            if(PerVertex==true){
               PerVertex=!PerVertex;
               gl.useProgram(program);
               document.getElementById("ButtonPerVertexPerFragment").innerHTML="Activate Per-Vertex (Deactivating Per-Fragment)";
               gl.uniform1f(gl.getUniformLocation(program, "uPerVertex"), PerVertex);
               gl.useProgram(program2);
               gl.uniform1f(gl.getUniformLocation(program2, "uPerVertex"), PerVertex);
               console.log("Automatic Deactivating of the Vertex Shader because this is a Per-Fragment feature");
            }
         }
         console.log("flagRoughTexture = "+flagRoughTexture);
    }

    // BASIC SOLID
    var ambientProductSolLightOrig = mult(lightAmbient, materialAmbient);
    var diffuseProductSolLightOrig = mult(lightDiffuse, materialDiffuse);
    var specularProductSolLightOrig = mult(lightSpecular, materialSpecular);

    var ambientProductSolNeon = mult(lightAmbientCyl, materialAmbient);
    var diffuseProductSolNeon = mult(lightDiffuseCyl, materialDiffuse);
    var specularProductSolNeon = mult(lightSpecularCyl, materialSpecular);
    
    // CYLINDER
    var ambientProductCylLightOrig = mult(lightAmbient, materialAmbientCyl);
    var diffuseProductCylLightOrig = mult(lightDiffuse, materialDiffuseCyl);
    var specularProducCyltLightOrig = mult(lightSpecular, materialSpecularCyl);

    var ambientProductCylNeon = mult(lightAmbientCyl, materialAmbientCyl);
    var diffuseProductCylNeon = mult(lightDiffuseCyl, materialDiffuseCyl);
    var specularProductCylNeon = mult(lightSpecularCyl, materialSpecularCyl);


    gl.useProgram(program);

    ///////PER-VERTEX_PER-FRAGMENT///////
    gl.uniform1f(gl.getUniformLocation(program, "uPerVertex"), PerVertex);
    /////////////////////////////////////

    modelViewMatrixLoc = gl.getUniformLocation(program, "uModelViewMatrix");
    projectionMatrixLoc = gl.getUniformLocation(program, "uProjectionMatrix");
    nMatrixLoc = gl.getUniformLocation(program, "uNormalMatrix");

    gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition"), lightPosition);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), ambientProductSolLightOrig);
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), diffuseProductSolLightOrig);
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProductSolLightOrig);
    gl.uniform1f(gl.getUniformLocation(program, "uShininess"), materialShininess);
    //////// CYLINDER ////////
    gl.uniform1i( gl.getUniformLocation(program, "uNeonLight"), flagNeonLight);
    gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProductCyl"), ambientProductSolNeon);
    gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProductCyl"), diffuseProductSolNeon);
    gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProductCyl"), specularProductSolNeon);
    gl.uniform1f(gl.getUniformLocation(program, "uShininessCyl"), materialShininessCyl);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPositionCyl1"), lightPositionCyl1);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPositionCyl2"), lightPositionCyl2);
    gl.uniform4fv(gl.getUniformLocation(program, "uLightPositionCyl3"), lightPositionCyl3);
    //////////////////////////


    gl.useProgram(program2);

    ///////PER-VERTEX_PER-FRAGMENT///////
    gl.uniform1f(gl.getUniformLocation(program2, "uPerVertex"), PerVertex);
    /////////////////////////////////////

    modelViewMatrixLocCylinder = gl.getUniformLocation(program2, "uModelViewMatrix");
    projectionMatrixLocCylinder = gl.getUniformLocation(program2, "uProjectionMatrix");
    nMatrixLocCylinder = gl.getUniformLocation(program2, "uNormalMatrix");

    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPosition"), lightPosition);
    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProduct"), ambientProductCylLightOrig);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProduct"), diffuseProductCylLightOrig);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProduct"), specularProducCyltLightOrig);
    gl.uniform1f(gl.getUniformLocation(program2, "uShininess"), materialShininess);
    gl.uniform4fv(gl.getUniformLocation(program2, "uEmissiveCyl"), emissiveCyl);
    //////// CYLINDER ////////
    gl.uniform1i( gl.getUniformLocation(program2, "uNeonLight"), flagNeonLight);
    gl.uniform4fv(gl.getUniformLocation(program2, "uAmbientProductCyl"), ambientProductCylNeon);
    gl.uniform4fv(gl.getUniformLocation(program2, "uDiffuseProductCyl"), diffuseProductCylNeon);
    gl.uniform4fv(gl.getUniformLocation(program2, "uSpecularProductCyl"), specularProductCylNeon);
    gl.uniform1f(gl.getUniformLocation(program2, "uShininessCyl"), materialShininessCyl);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPositionCyl1"), lightPositionCyl1);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPositionCyl2"), lightPositionCyl2);
    gl.uniform4fv(gl.getUniformLocation(program2, "uLightPositionCyl3"), lightPositionCyl3);
    //////////////////////////

    render();
}

function render(){
   gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
   
   gl.useProgram(program);

   if(flag && directionRotation){
      thetaRot[axis] += 2.0;
   }else if(flag && !directionRotation){
      thetaRot[axis] -= 2.0;
   }

   viewerPos = vec3(radius*Math.sin(thetaEx3)*Math.cos(phi), radius*Math.sin(thetaEx3)*Math.sin(phi), radius*Math.cos(thetaEx3));
   projectionMatrix = perspective(fovy, aspect, near, far);

   modelViewMatrix = lookAt(viewerPos, at , up);
   ///////// BARYCENTER //////////
   modelViewMatrix = mult(Trans, modelViewMatrix);
   ///////////////////////////////
   modelViewMatrix = mult(modelViewMatrix, rotate(thetaRot[xAxis], vec3(1, 0, 0)));
   modelViewMatrix = mult(modelViewMatrix, rotate(thetaRot[yAxis], vec3(0, 1, 0)));
   modelViewMatrix = mult(modelViewMatrix, rotate(thetaRot[zAxis], vec3(0, 0, 1)));
   ///////// BARYCENTER //////////
   modelViewMatrix = mult(modelViewMatrix, inverse(Trans));
   ///////////////////////////////
   nMatrix = normalMatrix(modelViewMatrix, true);

   gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix));
   gl.uniformMatrix4fv(projectionMatrixLoc, false, flatten(projectionMatrix));
   gl.uniformMatrix3fv(nMatrixLoc, false, flatten(nMatrix));

   gl.drawArrays(gl.TRIANGLES, 0, numPositions);

   
   gl.useProgram(program2);

   modelViewMatrixCylinder = lookAt(viewerPos, at , up);
   projectionMatrixCylinder = perspective(fovy, aspect, near, far);
   nMatrixCylinder = normalMatrix(modelViewMatrixCylinder, true);

   gl.uniformMatrix4fv(modelViewMatrixLocCylinder, false, flatten(modelViewMatrixCylinder));
   gl.uniformMatrix4fv(projectionMatrixLocCylinder, false, flatten(projectionMatrixCylinder));
   gl.uniformMatrix3fv(nMatrixLocCylinder, false, flatten(nMatrixCylinder));

   gl.drawArrays(gl.TRIANGLES, numPositions, ncylinder);

   requestAnimationFrame(render); 
}

}

shadedCube();
