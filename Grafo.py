from Vertice import Vertice
import Grafo
from collections import deque
from typing import Dict, Set, List, Any

class Grafo:
    """
    Classe que representa um grafo, podendo ser direcionado ou não.
    Utiliza lista de adjacência para armazenar as arestas.
    """

    def __init__(self, direcionado: bool = False) -> None:
        """
        Inicializa um grafo vazio.

        :param direcionado: Indica se o grafo é direcionado (True) ou não (False).
        """
        self.direcionado: bool = direcionado
        self._num_arestas: int = 0
        self.mapa_vertices: Dict[int, Vertice] = {}
        self.lista_adjacencia: Dict[int, Set[int]] = {}

    def __repr__(self) -> str:
        """Retorna uma representação textual do grafo."""
        if not self.lista_adjacencia:
            return "Grafo(V=0, E=0)"
        repr_str = ""
        for v, vizinhos in self.lista_adjacencia.items():
            repr_str += f"  {v} -> {sorted(list(vizinhos))}\n"
        return f"Grafo(V={self.get_numero_vertices()}, E={self.get_numero_arestas()}):\n{repr_str.strip()}"

    def get_numero_vertices(self) -> int:
        """Retorna o número de vértices do grafo."""
        return len(self.lista_adjacencia)

    def get_numero_arestas(self) -> int:
        """Retorna o número de arestas do grafo."""
        return self._num_arestas

    def get_vertices(self) -> List[int]:
        """Retorna a lista de vértices presentes no grafo."""
        return list(self.lista_adjacencia.keys())

    def inserir_vertice(self, v: int) -> None:
        """Insere um vértice no grafo, caso ainda não exista."""
        if v not in self.lista_adjacencia:
            self.lista_adjacencia[v] = set()
            self.mapa_vertices[v] = Vertice(v)

    def inserir_lista_vertices(self, vertices: List[int]) -> None:
        """Insere múltiplos vértices no grafo."""
        for v in vertices:
            self.inserir_vertice(v)

    def inserir_aresta(self, u: int, v: int, insere_vertices : bool = True) -> None:
        """
        Insere uma aresta entre os vértices u e v.

        :param u: Vértice de origem.
        :param v: Vértice de destino.
        """
        if(insere_vertices):
          self.inserir_vertice(u)
          self.inserir_vertice(v)

        if v not in self.lista_adjacencia[u]:
            self.lista_adjacencia[u].add(v)
            if not self.direcionado:
                self.lista_adjacencia[v].add(u)
            self._num_arestas += 1

    def remover_aresta(self, u: int, v: int) -> None:
        """
        Remove uma aresta entre os vértices u e v, se existir.

        :param u: Vértice de origem.
        :param v: Vértice de destino.
        """
        if self.aresta_existe(u, v):
            self.lista_adjacencia[u].remove(v)
            if not self.direcionado:
                self.lista_adjacencia[v].remove(u)
            self._num_arestas -= 1

    def aresta_existe(self, u: int, v: int) -> bool:
        """
        Verifica se existe uma aresta entre u e v.

        :param u: Vértice de origem.
        :param v: Vértice de destino.
        :return: True se a aresta existir, False caso contrário.
        """
        if u not in self.lista_adjacencia:
            return False
        return v in self.lista_adjacencia[u]

    def get_adjacentes(self, v_id: int) -> List[int]:
        """
        Retorna a lista de vértices adjacentes a um vértice.

        :param v_id: Identificador do vértice.
        :return: Lista de vértices adjacentes.
        """
        return list(self.lista_adjacencia.get(v_id, set()))

    def is_direcionado(self) -> bool:
        """Retorna True se o grafo for direcionado."""
        return self.direcionado

    def imprimir_caminho(self, origem_id: int, destino_id: int, caminho: List[int] = []) -> None:
        """
        Imprime o caminho entre dois vértices após a execução de um algoritmo de busca.

        :param origem_id: Vértice de origem.
        :param destino_id: Vértice de destino.
        :param caminho: Lista usada internamente para reconstruir o caminho.
        """
        self.bfs(origem_id)
        v_destino = self.mapa_vertices[destino_id]
        if v_destino.get_antecessor() != -1 and v_destino.get_antecessor() != origem_id:
            v_antecessor: Vertice = self.mapa_vertices[v_destino.get_antecessor()]
            caminho.append(v_antecessor.index)
            self.imprimir_caminho(origem_id, v_antecessor.index)
        else:
            caminho.append(origem_id)
            caminho.reverse()
            for v in caminho:
                print(v)

    def reseta_busca(self) -> None:
        """Reseta o estado de todos os vértices para permitir uma nova busca."""
        for v_obj in self.mapa_vertices.values():
            v_obj.desmarcar()
            v_obj.set_antecessor(-1)
            v_obj.tempo_d = 0
            v_obj.tempo_f = 0

    def _dfs(self, v: Vertice) -> None:
        """Executa a DFS recursiva a partir de um vértice."""
        v.marcar()
        for u in self.get_adjacentes(v.index):
            u = self.mapa_vertices[u]
            if not u.esta_marcado():
                u.set_antecessor(v)
                self._dfs(u)

    def dfs_recursiva(self) -> None:
        """Executa a DFS recursiva em todos os vértices do grafo."""
        self.reseta_busca()
        for v_tupla in self.lista_adjacencia.items():
            if v_tupla:
                v = self.mapa_vertices[v_tupla[0]]
                if not v.esta_marcado():
                    self._dfs(v)

    def dfs(self, v: int) -> None:
        """Executa a DFS iterativa a partir de um vértice."""
        self.reseta_busca()
        pilha: deque[int] = deque()
        v_objeto = self.mapa_vertices[v]
        pilha.append(v)
        while len(pilha) > 0:
            v = pilha.pop()
            v_objeto = self.mapa_vertices[v]
            if not v_objeto.esta_marcado():
                v_objeto.marcar()
                for u in self.get_adjacentes(v_objeto.index):
                    u_objeto = self.mapa_vertices[u]
                    if u_objeto.get_antecessor() == -1:
                        u_objeto.set_antecessor(v)
                        pilha.append(u)

    def _dfs_com_tempo(self, v: Vertice) -> None:
        """Executa a DFS registrando tempos de descoberta e finalização."""
        v.marcar()
        self.tempo += 1
        v.tempo_d = self.tempo
        for u in self.get_adjacentes(v.index):
            u = self.mapa_vertices[u]
            if not u.esta_marcado():
                u.set_antecessor(v)
                self._dfs_com_tempo(u)  
        self.tempo += 1
        v.tempo_f = self.tempo
        self.resultados.append((v.tempo_f, v.index))

    def dfs_com_tempo(self, id_v_inicial:int = None) -> list[(int, int)]:
        """
        Executa a DFS em todos os vértices e registra tempos de descoberta e finalização.

        :return: Dicionário com os tempos de descoberta e finalização de cada vértice.
        """
        
        self.reseta_busca()
        self.tempo = 0
        
        #resultados guardados como (tempo de finalização, id do vertice)
        self.resultados: list[tuple[int,Vertice.index]] = []
        
        if id_v_inicial is not None:
          v_inicial : Vertice = self.mapa_vertices[id_v_inicial]
          if not v_inicial.esta_marcado():
            self._dfs_com_tempo(v_inicial)
            
        else:
          for v_id in self.get_vertices():
            v : Vertice = self.mapa_vertices[v_id]
            if not v.esta_marcado():
              self._dfs_com_tempo(v)
            
        return self.resultados

    def bfs(self, v: int) -> None:
        """Executa a busca em largura (BFS) a partir de um vértice."""
        self.reseta_busca()
        fila: deque[int] = deque()
        v_objeto = self.mapa_vertices[v]
        fila.append(v)
        while len(fila) > 0:
            v = fila.popleft()
            v_objeto = self.mapa_vertices[v]
            for u in self.get_adjacentes(v_objeto.index):
                u_objeto = self.mapa_vertices[u]
                if u_objeto.get_antecessor() == -1:
                    u_objeto.set_antecessor(v)
                    u_objeto.marcar()
                    fila.append(u)

    def transposto(self) -> Grafo: 
      
      """retorna o grafo transposto(com arestas invertidas)"""

      if (not self.direcionado):
          return
        
      grafo_transposto = Grafo(direcionado=True)
      for id_v in self.lista_adjacencia.keys():
        for id_u in self.lista_adjacencia[id_v]:
          grafo_transposto.inserir_aresta(id_u, id_v)
          
      return grafo_transposto
  
    def kosaraju(self)-> List:
      
      """Executa o algoritmo kosaraju-sharir e retorna as componentes fortemente conexas do grafo"""
      
      componentes : list = []
      #roda uma dfs no grafo e computa os tempos de finalização
      tempos_dfs : list[(int,Vertice.index)] = self.dfs_com_tempo()
      grafo_reverso : Grafo = self.transposto()
      visitados : set[int] = set()
      self.reseta_busca()
      #roda a dfs no G transposto, só que a pilha de execução é determinada pela dfs anterior
      while len(tempos_dfs) > 0:
        
        v = tempos_dfs.pop()[1]  # pega o vértice com maior tempo de término
        if v not in visitados:
            componente = []
            pilha = [v]

            # DFS iterativa no grafo transposto
            while pilha:
                atual = pilha.pop()
                if atual not in visitados:
                    visitados.add(atual)
                    componente.append(atual)

                    # adiciona todos os adjacentes não visitados
                    for vizinho in grafo_reverso.get_adjacentes(atual):
                        if vizinho not in visitados:
                            pilha.append(vizinho)

            componentes.append(componente)

      
      #acho que tem problema na logica de direcionamento  
      return componentes
    #Prim
    #Kruskal
    #Dijkstra
    #Bellman-Ford
    #Capacity-scaling
    #Ford-Fulkerson
