"""
Inference Interface for the UniAD Model.
Handles preprocessing of 6 surround images, forward pass through UniAD,
and extraction of ego-vehicle planning waypoints.

Command encoding (matches UniAD/bev_render.py line 253):
  0 = TURN RIGHT
  1 = TURN LEFT
  2 = KEEP FORWARD (straight)
"""

import os
import torch
import numpy as np
from typing import Dict, Optional, Tuple

class UniADModelWrapper:
    """Wraps UniAD for real-time inference with fallback for dry-run testing."""
    
    def __init__(self, config_path: str, checkpoint_path: str, device: str = "cuda"):
        self.device = device if torch.cuda.is_available() and device == "cuda" else "cpu"
        self.is_ready = False
        self.model = None
        # BEVFormer temporal state: must be carried across ticks
        self.prev_bev = None

        print(f"[UniAD Interface] Initializing with device: {self.device}")
        
        # Check if model files exist
        if os.path.exists(config_path) and os.path.exists(checkpoint_path):
            try:
                self._load_model(config_path, checkpoint_path)
            except Exception as e:
                print(f"[UniAD Interface] Notice: Full CUDA model load deferred ({e}).")
                print(f"[UniAD Interface] Running in fallback/simulation-test mode.")
        else:
            print(f"[UniAD Interface] Checkpoint not found at '{checkpoint_path}'.")
            print(f"[UniAD Interface] Operating in mock-agent mode for pipeline testing.")

    def _load_model(self, config_path: str, checkpoint_path: str):
        """Loads UniAD model weights using mmcv/mmdet3d."""
        from mmcv import Config
        from mmdet3d.models import build_model
        from mmcv.runner import load_checkpoint

        cfg = Config.fromfile(config_path)
        model = build_model(cfg.model, train_cfg=None, test_cfg=cfg.get('test_cfg'))
        load_checkpoint(model, checkpoint_path, map_location=self.device)
        model.to(self.device)
        model.eval()
        self.model = model
        self.is_ready = True
        print("[UniAD Interface] UniAD weights loaded successfully!")

    def reset(self):
        """Clear BEVFormer temporal state. Call between CARLA episodes."""
        self.prev_bev = None

    def preprocess_images(self, images: Dict[str, np.ndarray]) -> torch.Tensor:
        """
        Preprocesses 6 RGB images into a batched tensor.
        Input: Dict of 6 (H, W, 3) arrays
        Output: Tensor of shape (1, 6, 3, H, W) normalized to ImageNet stats.
        """
        cam_order = [
            'CAM_FRONT', 'CAM_FRONT_RIGHT', 'CAM_FRONT_LEFT',
            'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT'
        ]
        
        img_list = []
        # Standard normalization: mean=[123.675, 116.28, 103.53], std=[58.395, 57.12, 57.375]
        mean = np.array([123.675, 116.28, 103.53], dtype=np.float32)
        std = np.array([58.395, 57.12, 57.375], dtype=np.float32)

        for cam in cam_order:
            img = images.get(cam, np.zeros((900, 1600, 3), dtype=np.uint8))
            # Resize if necessary to UniAD input size (e.g. 900x1600 or scaled)
            img_norm = (img.astype(np.float32) - mean) / std
            img_trans = img_norm.transpose(2, 0, 1)  # HWC to CHW
            img_list.append(img_trans)

        img_tensor = torch.tensor(np.stack(img_list), dtype=torch.float32).unsqueeze(0)
        return img_tensor.to(self.device)

    @torch.no_grad()
    def predict_waypoints(
        self, 
        images: Dict[str, np.ndarray], 
        ego_speed_kmh: float,
        command: int = 2  # UniAD encoding: 0=TURN RIGHT, 1=TURN LEFT, 2=KEEP FORWARD
    ) -> np.ndarray:
        """
        Runs inference and returns predicted waypoints.
        
        Command encoding (IMPORTANT - matches UniAD bev_render.py):
          0 = TURN RIGHT
          1 = TURN LEFT
          2 = KEEP FORWARD

        Returns:
            waypoints: np.ndarray of shape (6, 2) representing (x, y) relative offsets
                       for future horizons (0.5s, 1.0s, 1.5s, 2.0s, 2.5s, 3.0s).
        """
        if self.is_ready and self.model is not None:
            # Full model inference
            img_tensor = self.preprocess_images(images)
            speed_ms = ego_speed_kmh / 3.6

            # Build CAN bus metadata - 18 element vector with correct slot assignments:
            #  [0:3]   accel xyz,  [3:6] gyro xyz,  [6:10] orientation quaternion (w,x,y,z)
            #  [10:13] pose delta translation,  [13]  heading rate (rad/s)
            #  [14]    velocity forward (m/s),  [15]  velocity lateral (m/s)
            #  [16]    heading (rad),  [17] steering wheel angle
            can_bus = np.zeros(18, dtype=np.float32)
            can_bus[14] = speed_ms   # Forward velocity (m/s) - CORRECT slot
            can_bus[15] = 0.0        # Lateral velocity (m/s)

            dummy_meta = {
                'can_bus': can_bus,
                'img_shape': [(900, 1600, 3)] * 6,
                'command': command,
                'prev_bev': self.prev_bev,
                'use_prev_bev': self.prev_bev is not None,
            }

            outputs = self.model(return_loss=False, rescale=True, img=[img_tensor], img_metas=[[dummy_meta]])
            
            # Store BEVFormer temporal state for next tick
            if isinstance(outputs, dict) and 'bev_embed' in outputs:
                self.prev_bev = outputs['bev_embed']
            elif isinstance(outputs, list) and len(outputs) > 0 and 'bev_embed' in outputs[0]:
                self.prev_bev = outputs[0]['bev_embed']

            result = outputs[0] if isinstance(outputs, list) else outputs
            if 'planning_traj' in result:
                return result['planning_traj'].cpu().numpy()
            elif 'sdc_planning' in result:
                return result['sdc_planning'].cpu().numpy()

        # Fallback / Mock trajectory generator for testing the bridge
        # command encoding: 0=TURN RIGHT, 1=TURN LEFT, 2=KEEP FORWARD
        speed_ms = max(ego_speed_kmh / 3.6, 5.0)  # at least 5 m/s forward intent
        time_steps = np.array([0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
        x = speed_ms * time_steps  # forward distances

        if command == 1:    # TURN LEFT
            y = 0.5 * np.linspace(0.5, 4.0, 6)
        elif command == 0:  # TURN RIGHT
            y = -0.5 * np.linspace(0.5, 4.0, 6)
        else:               # KEEP FORWARD
            y = np.zeros(6)

        return np.stack([x, y], axis=1)
