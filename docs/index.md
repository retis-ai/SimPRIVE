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

1. Launch the ROS 2 TCP server:

    ```bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```

2. Ensure a ROS node is publishing to `/odom`.
3. Click **PLAY** in Unreal Engine – everything should work!

---

## 📚 Citation

To cite this work, use the following BibTeX entry:

<pre>
@article{example,
  title={Title of the Academic Paper},
  author={Nesti, Federico},
  journal={Journal Name},
  volume={XX},
  number={YY},
  pages={ZZZ--ZZZ},
  year={2025},
  publisher={Publisher},
  doi={10.1234/example.doi}
}
</pre>

---

<style>
.video-container {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  max-width: 100%;
}
.video-container iframe, .video-container object, .video-container embed {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}
.gallery {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  justify-content: center;
}
</style>
