import os
import torch.nn as nn
import torch
import torchvision
import torchvision.transforms as transforms
from glob import glob
import os
from torch.utils.data import Dataset, DataLoader
from torchvision.datasets.folder import default_loader
from tqdm import tqdm
import numpy as np


class base_dataset(Dataset):
    def __init__(self, data_list, classes, transform):
        self.data_list = data_list
        self.transform = transform
        self.class_dict = {cls_name: ix for ix, cls_name in enumerate(classes)}

    def __len__(self):
        return len(self.data_list)
    
    def __getitem__(self, index):
        # Load Image
        img_path = self.data_list[index]
        img = default_loader(img_path)
        img = self.transform(img)
        
        # Load Label
        label = self.class_dict[img_path.split('/')[-2]]
        label = torch.tensor(label, dtype=torch.long)
        return img, label



class basemodel(nn.Module):
    def __init__(self, num_classes):
        super(basemodel, self).__init__()
        self.net = torchvision.models.resnet50(num_classes=num_classes)
        self.transform = None
        self.classes = []
        self.meta = None
    
    def forward(self, image, check_ood, device):
        assert self.transform is not None
        assert len(self.classes) > 0
        
        self.net.to(device)
        
        image = self.transform(image)
        image = image.unsqueeze(0).to(device)
        
        out = self.net(image)
        out = torch.softmax(out, dim=1)
        confidence, pred = torch.max(out, 1)
        confidence = confidence[0]
                    
        
        if (check_ood) and (confidence < 0.6):
            label = 'OOD'
        else:
            label = self.classes[pred[0]]
        
        return confidence, label


    def finetune(self, data_list, strategy='classifier_tuning', epoch=10, lr=1e-4, batch_size=64, ratio=1.0, device='cpu'):
        # update classes and meta info
        self.classes = [d_ix.split('/')[-1] for d_ix in data_list]
        self.meta['classes'] = ['/'.join(d_ix.split('/')[-2:]) for d_ix in data_list]
        
        # change fc layer
        in_features = self.net.fc.in_features
        self.net.fc = nn.Linear(in_features, len(self.classes))
        
        # load optimizer
        if strategy == 'classifier_tuning':
            for param in self.net.parameters():
                param.requires_grad = False
            
            for param in self.net.fc.parameters():
                param.requires_grad = True
            
        model_param = [{'params': [p for p in self.net.parameters() if p.requires_grad]}]
        optimizer = torch.optim.SGD(model_param, lr=lr, weight_decay=5e-3, momentum=0.9)
        criterion = nn.CrossEntropyLoss()
        
        # dataset loader
        data_dict = {dl.split('/')[-1] : glob(os.path.join(dl, '*.png')) +\
                                         glob(os.path.join(dl, '*.jpg')) +\
                                         glob(os.path.join(dl, '*.JPEG')) for dl in data_list}
        
        
        train_data_list = []
        for data_ix in data_dict.values():
            train_data_list += data_ix
        train_data_list = np.random.choice(train_data_list, int(len(train_data_list) * ratio), replace=False).tolist()

        tr_dataset = base_dataset(train_data_list, self.classes, self.transform)
        tr_loader = DataLoader(tr_dataset, batch_size=batch_size, shuffle=True, drop_last=False)
        
        # trainer
        self.net.to(device)
        self.net.train()
        
        for current_epoch in range(epoch):
            tr_loss = 0.
            for image, label in tqdm(tr_loader):
                image, label = image.to(device), label.to(device)
                
                pred = self.net(image)
                loss = criterion(pred, label)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                tr_loss += loss.item()
            
            tr_loss /= len(tr_loader)
            print('Epoch(%d/%d) - tr_loss:%.3f' %(current_epoch + 1, epoch, tr_loss))

        self.net.eval() 
        
    
def load_model(num_classes):
    model = basemodel(num_classes)
    return model


def load_pretrained_model(base_folder, api_name, num_classes):
    model = load_model(num_classes)
    model.net.load_state_dict(torch.load(os.path.join(base_folder, api_name, 'model.pt'), map_location='cpu'))
    return model


def load_transform():
    transform = transforms.Compose([
            transforms.Resize((256,256)),
            transforms.CenterCrop((224,224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                std= [0.229, 0.224, 0.225]) 
        ])
    return transform
