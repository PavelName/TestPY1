import torch, torch.nn as nn, torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

# --- Данные ---
transform = transforms.Compose([transforms.ToTensor()])
train_loader = DataLoader(
    datasets.CIFAR10(root='./data', train=True, transform=transform, download=True),
    batch_size=64,
    shuffle=True
)
test_loader = DataLoader(
    datasets.CIFAR10(root='./data', train=False, transform=transform, download=True),
    batch_size=64
)


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
criterion = nn.CrossEntropyLoss()

# --- Модели ---
class SimpleCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.pool, self.relu = nn.MaxPool2d(2, 2), nn.ReLU()
        self.fc = nn.Linear(32 * 8 * 8, 10)
    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        return self.fc(x.view(x.size(0), -1))

class ImprovedCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.conv3 = nn.Conv2d(32, 64, 3, padding=1)
        self.pool, self.relu = nn.MaxPool2d(2, 2), nn.ReLU()
        self.fc = nn.Linear(64 * 4 * 4, 10)
    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        x = self.pool(self.relu(self.conv3(x)))
        return self.fc(x.view(x.size(0), -1))

# --- Обучение с сохранением метрик ---
def train_with_metrics(model, name, epochs=10):
    model = model.to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    train_losses, val_losses = [], []
    train_accs, val_accs = [], []

    for e in range(epochs):
        # Train
        model.train()
        running_loss, correct, total = 0, 0, 0
        for im, lb in train_loader:
            im, lb = im.to(device), lb.to(device)
            optimizer.zero_grad()
            out = model(im)
            loss = criterion(out, lb)
            loss.backward(); optimizer.step()
            running_loss += loss.item()
            _, pred = torch.max(out, 1)
            total += lb.size(0); correct += (pred == lb).sum().item()
        train_losses.append(running_loss / len(train_loader))
        train_accs.append(100 * correct / total)

        # Val
        model.eval()
        val_loss, correct_val, total_val = 0, 0, 0
        with torch.no_grad():
            for im, lb in test_loader:
                im, lb = im.to(device), lb.to(device)
                out = model(im)
                loss = criterion(out, lb)
                val_loss += loss.item()
                _, pred = torch.max(out, 1)
                total_val += lb.size(0)
                correct_val += (pred == lb).sum().item()
        val_losses.append(val_loss / len(test_loader))
        val_accs.append(100 * correct_val / total_val)

        print(f"{name} | Epoch {e+1}: Train Loss={train_losses[-1]:.4f}, Val Loss={val_losses[-1]:.4f}, "
              f"Train Acc={train_accs[-1]:.2f}%, Val Acc={val_accs[-1]:.2f}%")

    return train_losses, val_losses, train_accs, val_accs

# --- Запуск обеих моделей ---
epochs = 15
tl_s, vl_s, ta_s, va_s = train_with_metrics(SimpleCNN(), "SimpleCNN", epochs)
tl_i, vl_i, ta_i, va_i = train_with_metrics(ImprovedCNN(), "ImprovedCNN", epochs)

# --- Построение графиков ---
fig, axs = plt.subplots(1, 2, figsize=(14, 5))

# Loss
axs[0].plot(tl_s, label="SimpleCNN Train", linestyle='--')
axs[0].plot(vl_s, label="SimpleCNN Val")
axs[0].plot(tl_i, label="ImprovedCNN Train", linestyle='--', color='orange')
axs[0].plot(vl_i, label="ImprovedCNN Val", color='red')
axs[0].set_title("Loss: Train vs Validation")
axs[0].set_xlabel("Epoch")
axs[0].set_ylabel("Loss")
axs[0].legend()
axs[0].grid(True, alpha=0.3)

# Accuracy
axs[1].plot(ta_s, label="SimpleCNN Train", linestyle='--')
axs[1].plot(va_s, label="SimpleCNN Val")
axs[1].plot(ta_i, label="ImprovedCNN Train", linestyle='--', color='orange')
axs[1].plot(va_i, label="ImprovedCNN Val", color='red')
axs[1].set_title("Accuracy: Train vs Validation")
axs[1].set_xlabel("Epoch")
axs[1].set_ylabel("Accuracy (%)")
axs[1].legend()
axs[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
