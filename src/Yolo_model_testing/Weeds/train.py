from ultralytics import YOLO
import matplotlib.pyplot as plt
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# Initialize the YOLOv8 model (this will load the default YOLOv8 model)
model = YOLO('yolov8n.pt')  # You can replace with yolov8s.pt or another variant

# Training the model
results = model.train(
    data='weeds_dataset/data.yaml',  # Path to your data.yaml file
    imgsz=640,  # Image size
    batch=32,  # Batch size
    epochs=100,  # Number of epochs
    device=device,  # Use GPU (0 for first GPU, or 'cpu' for CPU)
    project= 'trained_models',  # Directory to save training results
    name='weeds_model',  # Name of the experiment
)

# Accessing and plotting the training errors
train_loss = results.history['loss']
val_loss = results.history['val_loss']

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(range(1, 101), train_loss, label="Training Loss")
plt.plot(range(1, 101), val_loss, label="Validation Loss")
plt.title('YOLOv8 Training and Validation Loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.legend()
plt.savefig("yolov8_training_plot.png")  # Save plot as PNG file
plt.show()

# Print the final results for the last epoch
print(f"Training complete. Best results after 100 epochs: \n{results.best}")
