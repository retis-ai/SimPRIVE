---
layout: post
title: "Title of the Academic Paper"
author: "Federico Nesti"
date: 2025-07-30
categories: [research, academic]
tags: [jekyll, markdown, academic-paper]
---

<div class="text-center">
  <h1>Title of the Academic Paper</h1>
  <p class="lead">Subtitle or Short Description</p>
</div>

---

## 🧪 SimPRIVE

> SimPRIVE is a [short intro placeholder]. This section will describe the goals, components, and capabilities of SimPRIVE.

TODO: Add brief explanation of what SimPRIVE is.

---

## 📄 Research

- [Link to Paper (PDF)](your-link-here)
- [Code Repository (GitHub)](your-link-here)
- [Dataset (if available)](your-link-here)

---

## 🎬 Demo

<div class="video-container">
  <iframe width="560" height="315" src="https://www.youtube.com/embed/your_video_id" frameborder="0" allowfullscreen></iframe>
</div>

> _Watch the demo above showing SimPRIVE in action._

---

## 📸 Screenshots

<div class="gallery">
  <img src="/assets/images/simprive1.png" alt="SimPRIVE screenshot 1" width="45%">
  <img src="/assets/images/simprive2.png" alt="SimPRIVE screenshot 2" width="45%">
</div>

---

## 🛠️ Instructions

### 🔧 Prerequisites

- **UE 5.2** (Unreal Engine)
- **ROS 2 Humble** (Ubuntu 22)
- **[rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/ros2)**

### ⚙️ Setup Steps

1. Create a new UE5.2 project (use any desired template).
2. Download and place **SimPRIVE plugin** in your project's `Plugins` folder.
3. Download and place **[ROSIntegration plugin](https://github.com/code-iai/ROSIntegration)** in `Plugins`.
4. Enable both plugins in UE.
5. Edit the plugin JSON config to set the correct ROS **IP** and **PORT**.
6. Go to `Project Settings → Maps & Modes → Game Instance Class` and set it to `SimPRIVEGameInstance`.

### 🌍 Level Setup

1. Create a new level.
2. **Set custom collision presets** (important for interactions).
3. Add the **rover**, define its height.
4. Add the **SpawningManager**, define **splines** and **object library**.
5. Add **ROsOrchestrator** and assign:
   - Rover
   - SpawningManager
   - ROS topic names

### 🔄 ROS 2 Side

1. Launch the ROS 2 TCP server with:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml

