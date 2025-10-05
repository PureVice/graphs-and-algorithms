class Vertice:
  
  def __init__(self, index): 
    
    self.index = index
    self.marcado = False
    self.antecessor = -1
    self.tempo_d = 0
    self.tempo_f = 0
  def __repr__(self): 
    
    if self.marcado == True :
      marca = 'S'
    else: 
      marca = 'N'
    representacao = f"""ID: {self.index} 
                    \n"""
    return representacao
  
  def marcar(self):
    self.marcado = True
    
  def desmarcar(self):
    self.marcado = False
  
  def esta_marcado(self):
    return self.marcado
  
  def set_antecessor(self, v):
    self.antecessor = v
  
  def get_antecessor(self):
    return self.antecessor
  