import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

# 1. Настройка устройства
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Используемое устройство: {device}")

# 2. Трансформации (Data Augmentation)
cifar10_mean = (0.4914, 0.4822, 0.4465)
cifar10_std = (0.2470, 0.2435, 0.2616)

train_transform = transforms.Compose([
    transforms.RandomCrop(32, padding=4),
    transforms.RandomHorizontalFlip(p=0.5),
    transforms.ToTensor(),
    transforms.Normalize(cifar10_mean, cifar10_std)
])

test_transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(cifar10_mean, cifar10_std)
])

# Загрузка данных
train_dataset = datasets.CIFAR10(root='./data', train=True, download=True, transform=train_transform)
test_dataset = datasets.CIFAR10(root='./data', train=False, download=True, transform=test_transform)

train_loader = DataLoader(train_dataset, batch_size=128, shuffle=True, num_workers=2)
test_loader = DataLoader(test_dataset, batch_size=128, shuffle=False, num_workers=2)

classes = ['plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']

# 3. Определение моделей

# Базовая модель (упрощенная, без BN и Dropout, для сравнения)
class BaseCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128 * 4 * 4, 128),
            nn.ReLU(),
            nn.Linear(128, 10)
        )
    def forward(self, x):
        x = self.features(x)
        return self.classifier(x)

# Улучшенная модель (с BatchNorm и Dropout)
class ImprovedCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, 3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Dropout(0.25),
            
            nn.Conv2d(32, 64, 3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            nn.Conv2d(64, 128, 3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Dropout(0.25)
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128 * 4 * 4, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128, 10)
        )
    def forward(self, x):
        x = self.features(x)
        return self.classifier(x)

# 4. Функция обучения и оценки
def train_and_evaluate(model_class, name, epochs=15):
    print(f"\n--- Запуск эксперимента: {name} ---")
    model = model_class().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.9, weight_decay=5e-4)
    
    history_loss = []
    history_acc = []
    
    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        
        for im, lb in train_loader:
            im, lb = im.to(device), lb.to(device)
            
            optimizer.zero_grad()
            outputs = model(im)
            loss = criterion(outputs, lb)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
        
        # Оценка на тесте
        model.eval()
        correct = 0
        total = 0
        with torch.no_grad():
            for im, lb in test_loader:
                im, lb = im.to(device), lb.to(device)
                outputs = model(im)
                _, predicted = torch.max(outputs.data, 1)
                total += lb.size(0)
                correct += (predicted == lb).sum().item()
        
        acc = 100 * correct / total
        avg_loss = running_loss / len(train_loader)
        
        history_loss.append(avg_loss)
        history_acc.append(acc)
        
        if (epoch + 1) % 5 == 0:
            print(f"Epoch [{epoch+1}/{epochs}] - Loss: {avg_loss:.4f}, Test Acc: {acc:.2f}%")
    
    final_acc = history_acc[-1]
    print(f"Итоговая точность ({name}): {final_acc:.2f}%")
    return final_acc, history_loss, history_acc

# 5. Проведение экспериментов
base_acc, base_loss, base_acc_hist = train_and_evaluate(BaseCNN, "Базовая модель (без оптимизаций)", epochs=15)
improved_acc, imp_loss, imp_acc_hist = train_and_evaluate(ImprovedCNN, "Улучшенная модель (BN + Dropout + Aug)", epochs=15)

# 6. Визуализация результатов
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(base_loss, label='Base Loss', linestyle='--')
plt.plot(imp_loss, label='Improved Loss')
plt.title('Динамика функции потерь (Loss)')
plt.xlabel('Эпоха')
plt.ylabel('Loss')
plt.legend()
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(base_acc_hist, label=f'Base Acc ({base_acc:.1f}%)', linestyle='--')
plt.plot(imp_acc_hist, label=f'Improved Acc ({improved_acc:.1f}%)')
plt.title('Динамика точности на тесте (Accuracy)')
plt.xlabel('Эпоха')
plt.ylabel('Accuracy (%)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

print(f"\n=== ИТОГОВОЕ СРАВНЕНИЕ ===")
print(f"Точность базовой модели:      {base_acc:.2f}%")
print(f"Точность улучшенной модели: {improved_acc:.2f}%")
print(f"Прирост точности:           {improved_acc - base_acc:.2f}%")
