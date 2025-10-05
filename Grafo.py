# Tipo abstrato de dados Grafo
#   Operações necessárias:

# *   Criar grafo vazio com |V| vértices e nenhuma aresta - check
# *   Inserir aresta no grafo - check
# *   Verificar se uma aresta pertence ao grafo - check
# *   Obter lista de vértices adjacentes a um dado vértice - check
# *   Imprimir grafo - check ?
# *   Remover aresta do grafo
# *   Obter número de vértices e arestas
# *   Determinar se grafo é direcionado ou não
import heapq

class Vertice:
  
  def __init__(self, index, marcado = False, antecessor = -1): 
    
    self.index = index
    self.marcado = marcado
    self.antecessor = antecessor
    
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
  
  
class Grafo:
  
  def __init__(self, direcionado=False):
    
    
    self.direcionado = direcionado
    self._num_arestas = 0
    self.mapa_id_obj = {}
    self.lista_adjacencia = {}  
    self.pilha = []
    
  def inserir_vertice(self, v):
    if v not in self.lista_adjacencia:
        self.lista_adjacencia[v] = set()
        self.mapa_id_obj[v] = Vertice(v) 
        
  def inserir_lista_vertices(self, vertices):
    for v in vertices:
      self.inserir_vertice(v)
      
    
  def inserir_aresta(self, u, v):
    # Garante que ambos os vértices existem antes de criar a aresta
    self.inserir_vertice(u)
    self.inserir_vertice(v)
    
    if v not in self.lista_adjacencia[u]:
        self.lista_adjacencia[u].add(v)
        if not self.direcionado:
            self.lista_adjacencia[v].add(u)
        self._num_arestas += 1

  def remover_aresta(self, u, v):
    if self.aresta_pertence(u, v):
        self.lista_adjacencia[u].remove(v)
        if not self.direcionado:
            self.lista_adjacencia[v].remove(u)
        self._num_arestas -= 1

  def aresta_pertence(self, u, v):
    if u not in self.lista_adjacencia:
        return False
    return v in self.lista_adjacencia[u]

  def get_adjacentes(self, v_id : int):
    """retorna uma lista de inteiros"""
    return list(self.lista_adjacencia.get(v_id, set()))

  
  def is_direcionado(self):
    return self.direcionado
  
  
  

  def imprimir_caminho(self, origem_id, destino_id, caminho=[]):
    
    self.dfs_iterativa(origem_id)
    """
    Imprime o caminho de um vértice de origem até um de destino,
    após a execução de um algoritmo de busca como DFS ou BFS.
    """
    v_destino = self.mapa_id_obj[destino_id]
    if v_destino.get_antecessor() != -1 and v_destino.get_antecessor() != origem_id:
      v_antecessor : Vertice = self.mapa_id_obj[v_destino.get_antecessor()]
      caminho.append(v_antecessor.index)
      self.imprimir_caminho(origem_id, v_antecessor.index)
    else:
      caminho.append(origem_id)
      caminho.reverse()
      
      for v in caminho: print(v)
      
  def reseta_busca(self):
    """Reseta o estado de todos os vértices para uma nova busca."""
    for v_obj in self.mapa_id_obj.values():
      v_obj.desmarcar()
      v_obj.set_antecessor(-1)
    
  
  def _dfs(self, v : Vertice):

    v.marcar()
      
    for u in self.get_adjacentes(v.index):
      u = self.mapa_id_obj[u]
      
      if not u.esta_marcado():
        u.set_antecessor(v)
        self._dfs(u)
          
  def dfs_recursiva(self):
    
    self.reseta_busca()
      
    for v_tupla in self.lista_adjacencia.items():
      if(v_tupla):
        
        v = self.mapa_id_obj[v_tupla[0]]
                
        if not v.esta_marcado():
          self._dfs(v)
          
    for v_tupla in self.lista_adjacencia.items():
      if(v_tupla):
        
        v = self.mapa_id_obj[v_tupla[0]]
        
            
  def __repr__(self):
    
    if not self.lista_adjacencia:
        return "Grafo(V=0, E=0)"
    
    repr_str = ""
    for v, vizinhos in self.lista_adjacencia.items():
        repr_str += f"  {v} -> {sorted(list(vizinhos))}\n"
    return f"Grafo(V={self.get_numero_vertices()}, E={self.get_numero_arestas()}):\n{repr_str.strip()}"

  def get_numero_vertices(self):
    return len(self.lista_adjacencia)

  def get_numero_arestas(self):
    return self._num_arestas

  def get_vertices(self):
    return list(self.lista_adjacencia.keys())

  
  def dfs_iterativa(self, v : int):
    self.reseta_busca()
    pilha = []
    v_objeto = self.mapa_id_obj[v]
    pilha.append(v)
    while (len(pilha)>0):
      v = pilha.pop()
      v_objeto = self.mapa_id_obj[v]
      if (not v_objeto.esta_marcado()):
        v_objeto.marcar()
        for u in self.get_adjacentes(v_objeto.index):
          u_objeto = self.mapa_id_obj[u]
          if (u_objeto.get_antecessor() == -1):
            u_objeto.set_antecessor(v)
            pilha.append(u)
    
    
          