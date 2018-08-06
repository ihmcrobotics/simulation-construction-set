---
title: YoAppearance helper API
---

Table 4 lists the methods in the YoAppearance helper API. To add a shape to a link, you simply use the corresponding method. 

For example, the following code will create a red sphere with radius of 0.3 meters and of the given mass and moment of inertia.

```java
Link link1 = new Link("link1");
link1.setMass(1.0);
link1.setMomentOfIntertia(0.1, 0.1, 0.1);
link1.setComOffset(0.0,0.0,0.0);
link1.addSphere(0.3, YoAppearance.Red());
```


Returns an Appearance with the given RGB values, each between 0.0 and 1.0
Appearance YoAppearance.RGBColor(float red, float green, float blue)

The user can choose among a set of predefined Appearances with the following syntax: 
```java
//Returns a black Appearance.
Appearance YoAppearance.Black()
//Returns a white Appearance.
Appearance YoAppearance.White()
//Returns a blue Appearance.
Appearance YoAppearance.Blue()
//...
```

The available colors are: White, Black, Blue, DarkBlue, Red, DarkRed, Green, DarkGreen, Silver, Gray, Marroon, Purple, Fushia, Olive, Yellow, navy, Teal, Aqua, BlackMetalMaterial, and AluminumMaterial.      

Sets the transparency of Appearance app to the desired value. 1.0 is fully transparent.
```java
void YoAppearance.makeTransparent(Appearance app, float transparency);
```

Sets the polygon attributes of Appearance app such that only edges are drawn.
```java
void makeLineDrawing(Appearance app);
```

Returns a texture mapped Earth Appearance. The Earth graphics are taken from the Java3D tutorials.
```java
Appearance YoAppearance.EarthTexture(Component comp);
```

Returns a texture mapped stone Appearance. The stone graphics are taken from the Java3D tutorials.
```java
Appearance YoAppearance.StoneTexture(Component comp);
```

### Related Example:

[Example of how YoAppearance is used](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-creating-links.html)