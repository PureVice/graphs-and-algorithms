import heapq
from DisjointSet import DisjointSet
from Vertice import Vertice
from collections import deque
from typing import Dict, Set, List, Tuple, Hashable

class Grafo:
    """
    Representa um grafo direcionado ou não direcionado.
    Usa dicionário de conjuntos para armazenar as adjacências.
    """

    def __init__(self, direcionado: bool = False) -> None:
        """Inicializa o grafo vazio."""
        self.direcionado: bool = direcionado
        self._num_arestas: int = 0
        self.mapa_vertices: Dict[Hashable, Vertice] = {}
        self.lista_adjacencia: Dict[Hashable, Set[Hashable]] = {}

    def __repr__(self) -> str:
        """Retorna representação textual do grafo."""
        if not self.lista_adjacencia:
            return "Grafo(V=0, E=0)"
        repr_str = "\n".join(
            f"  {v} -> {sorted(list(vizinhos))}"
            for v, vizinhos in self.lista_adjacencia.items()
        )
        return f"Grafo(V={self.get_numero_vertices()}, E={self.get_numero_arestas()}):\n{repr_str}"

    def get_numero_vertices(self) -> int:
        return len(self.lista_adjacencia)

    def get_numero_arestas(self) -> int:
        return self._num_arestas

    def get_vertices(self) -> List[Hashable]:
        return list(self.lista_adjacencia.keys())

    def inserir_vertice(self, v: Hashable) -> None:
        """Insere um vértice, caso ainda não exista."""
        if v not in self.lista_adjacencia:
            self.lista_adjacencia[v] = set()
            self.mapa_vertices[v] = Vertice(v)

    def inserir_lista_vertices(self, vertices: List[Hashable]) -> None:
        """Insere múltiplos vértices."""
        for v in vertices:
            self.inserir_vertice(v)

    def inserir_aresta(self, u: Hashable, v: Hashable, insere_vertices: bool = True) -> None:
        """Insere uma aresta entre u e v."""
        if insere_vertices:
            self.inserir_vertice(u)
            self.inserir_vertice(v)

        if v not in self.lista_adjacencia[u]:
            self.lista_adjacencia[u].add(v)
            if not self.direcionado:
                self.lista_adjacencia[v].add(u)
            self._num_arestas += 1

    def remover_aresta(self, u: Hashable, v: Hashable) -> None:
        """Remove uma aresta entre u e v, se existir."""
        if self.aresta_existe(u, v):
            self.lista_adjacencia[u].remove(v)
            if not self.direcionado:
                self.lista_adjacencia[v].remove(u)
            self._num_arestas -= 1

    def aresta_existe(self, u: Hashable, v: Hashable) -> bool:
        """Verifica se existe aresta entre u e v."""
        return u in self.lista_adjacencia and v in self.lista_adjacencia[u]

    def get_adjacentes(self, v_id: Hashable) -> List[Hashable]:
        """Retorna os vértices adjacentes a v_id."""
        return list(self.lista_adjacencia.get(v_id, set()))

    def reseta_busca(self) -> None:
        """Reseta marcações e tempos dos vértices."""
        for v_obj in self.mapa_vertices.values():
            v_obj.desmarcar()
            v_obj.set_antecessor(None)
            v_obj.tempo_d = 0
            v_obj.tempo_f = 0

    def _dfs(self, v: Vertice) -> None:
        """DFS recursiva a partir de v."""
        v.marcar()
        for u in self.get_adjacentes(v.index):
            u = self.mapa_vertices[u]
            if not u.esta_marcado():
                u.set_antecessor(v)
                self._dfs(u)

    def dfs_recursiva(self) -> None:
        """Executa DFS recursiva em todo o grafo."""
        self.reseta_busca()
        for v_id in self.get_vertices():
            v = self.mapa_vertices[v_id]
            if not v.esta_marcado():
                self._dfs(v)

    def dfs(self, v: Hashable) -> None:
        """DFS iterativa a partir de v."""
        self.reseta_busca()
        pilha: deque[Hashable] = deque([v])
        while pilha:
            atual = pilha.pop()
            v_obj = self.mapa_vertices[atual]
            if not v_obj.esta_marcado():
                v_obj.marcar()
                for u in self.get_adjacentes(atual):
                    u_obj = self.mapa_vertices[u]
                    if not u_obj.esta_marcado():
                        u_obj.set_antecessor(atual)
                        pilha.append(u)

    def bfs(self, v: Hashable) -> None:
        """BFS a partir de v."""
        self.reseta_busca()
        fila: deque[Hashable] = deque([v])
        while fila:
            atual = fila.popleft()
            v_obj = self.mapa_vertices[atual]
            for u in self.get_adjacentes(atual):
                u_obj = self.mapa_vertices[u]
                if not u_obj.esta_marcado():
                    u_obj.set_antecessor(atual)
                    u_obj.marcar()
                    fila.append(u)

    def _dfs_com_tempo(self, v: Vertice) -> None:
        """DFS registrando tempos de descoberta e término."""
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

    def dfs_com_tempo(self, id_v_inicial: Hashable = None) -> List[Tuple[int, Hashable]]:
        """Executa DFS completa e retorna lista de (tempo_fim, id_vértice)."""
        self.reseta_busca()
        self.tempo = 0
        self.resultados: List[Tuple[int, Hashable]] = []

        if id_v_inicial is not None:
            v_inicial = self.mapa_vertices[id_v_inicial]
            if not v_inicial.esta_marcado():
                self._dfs_com_tempo(v_inicial)
        else:
            for v_id in self.get_vertices():
                v = self.mapa_vertices[v_id]
                if not v.esta_marcado():
                    self._dfs_com_tempo(v)

        return self.resultados

    def transposto(self) -> "Grafo":
        """Retorna o grafo transposto (arestas invertidas)."""
        if not self.direcionado:
            raise ValueError("O grafo precisa ser direcionado para calcular o transposto.")

        g_t = Grafo(direcionado=True)
        for v in self.get_vertices():
            g_t.inserir_vertice(v)
        for u in self.lista_adjacencia:
            for v in self.lista_adjacencia[u]:
                g_t.inserir_aresta(v, u)
        return g_t

    def kosaraju(self) -> List[List[Hashable]]:
        """Executa Kosaraju-Sharir e retorna as componentes fortemente conexas."""
        componentes: List[List[Hashable]] = []

        # Passo 1: DFS normal -> tempos de finalização
        tempos_dfs = self.dfs_com_tempo()

        # Passo 2: Transpõe o grafo
        g_rev = self.transposto()
        visitados: Set[Hashable] = set()

        # Passo 3: DFS no transposto (em ordem decrescente de tempo)
        while tempos_dfs:
            _, v = tempos_dfs.pop()
            if v not in visitados:
                componente = []
                pilha = [v]
                while pilha:
                    atual = pilha.pop()
                    if atual not in visitados:
                        visitados.add(atual)
                        componente.append(atual)
                        for viz in g_rev.get_adjacentes(atual):
                            if viz not in visitados:
                                pilha.append(viz)
                componentes.append(sorted(componente))

        return componentes
    
class GrafoPonderado(Grafo):
    
    def __init__(self, direcionado=False):
        """cria um Grafo vazio, cujas arestas possuem pesos e vértices possuem rótulos"""
        super().__init__(direcionado)
        self.lista_adjacencia : Dict[Hashable, Dict[Hashable, float]] = {}
        
        # 'a' : {'b' : 2, 'c' : 3}
    
    def __repr__(self):
        """Retorna representação textual do grafo."""
        if not self.lista_adjacencia:
            return "Grafo(V=0, E=0)"
        repr_str = "\n".join(
            f"  {v} -> {sorted(list(vizinhos))}"
            for v, vizinhos in self.lista_adjacencia.items()
        )
        return f"Grafo(V={self.get_numero_vertices()}, E={self.get_numero_arestas()}):\n{repr_str}"

    def reseta_busca(self):
        for v_obj in self.mapa_vertices.values():
            v_obj.desmarcar()
            v_obj.set_antecessor(None)
            v_obj.tempo_d = 0
            v_obj.tempo_f = 0
            v_obj.rotulo = float('inf')
        
        
    def inserir_vertice(self,v) -> None:
        
        if v not in self.lista_adjacencia:
            self.lista_adjacencia[v] = {}
            self.mapa_vertices[v] = Vertice(v)
            
    def inserir_aresta(self, u, v, peso, insere_vertice = True) -> None:
        """insere uma aresta ponderada """
        if not insere_vertice:
            
            if u not in self.mapa_vertices.items() or v not in self.mapa_vertices.items():
                raise Exception() #qual exceção? return ?
            
        self.inserir_vertice(u)
        self.inserir_vertice(v)
            
        self.lista_adjacencia[u][v] = peso
        
        if(not self.direcionado):
            self.lista_adjacencia[v][u] = peso
                   
    def prim(self, v_inicial : int)-> Dict[Hashable, float]: 
        """retorna a árvore geradora mínima"""
        self.reseta_busca()
        arvore_minima : Dict = {}
        self.min_heap : List [List[float, Hashable]] = [] # (rotulo, id)
        heapq.heapify(self.min_heap)
        
        for v in self.lista_adjacencia:           
            if v != v_inicial:
                heapq.heappush(self.min_heap, [float('inf'), v ])
            else: 
                heapq.heappush(self.min_heap, [0.0, v ])
                v_inicial_obj : Vertice = self.mapa_vertices[v_inicial]
                v_inicial_obj.rotulo = 0.0

        while self.min_heap:
            par_min = heapq.heappop(self.min_heap)
            min : Hashable = par_min[1]
            rotulo_min : float = par_min[0] 
            
            min_obj = self.mapa_vertices[min]
            if min_obj.esta_marcado():
                continue # já processado antes (duplicata do heap)
            min_obj.marcar()
            arvore_minima[min] = (min_obj.antecessor, rotulo_min)

            for v in self.get_adjacentes(min):
                v_obj = self.mapa_vertices[v]
                peso_aresta = self.lista_adjacencia[min][v]
                if not v_obj.esta_marcado() and peso_aresta < v_obj.rotulo:
                    
                    v_obj.antecessor = min
                    v_obj.rotulo = peso_aresta
                    #self._atualiza_rotulo_heap(v, peso_aresta)
                    heapq.heappush(self.min_heap, [peso_aresta, v])
    
        return arvore_minima
    
    def kruskal(self, v_inicial: Hashable = None) -> Dict[Hashable, Tuple[Hashable, float]]:
        """
       retorna a árvore geradora mínima
        """

        arestas = []
        for u in self.lista_adjacencia:
            for v, peso in self.lista_adjacencia[u].items():
                if self.direcionado or u < v:  # evita duplicatas em grafos não-dirigidos
                    arestas.append((peso, u, v))
        arestas.sort()

        dsu = DisjointSet()
        for vertice in self.get_vertices():
            dsu.make_set(vertice)

        arvore_minima: Dict[Hashable, Tuple[Hashable, float]] = {}

        for peso, u, v in arestas:
            if dsu.union(u, v):
                if v not in arvore_minima and u in arvore_minima:
                    arvore_minima[v] = (u, peso)
                elif u not in arvore_minima and v in arvore_minima:
                    arvore_minima[u] = (v, peso)
                else:
                    arvore_minima[v] = (u, peso)

        if v_inicial is not None:
            raiz_inicial = dsu.find(v_inicial)
            arvore_minima = {
                v: (ant, peso)
                for v, (ant, peso) in arvore_minima.items()
                if dsu.find(v) == raiz_inicial
            }
            if v_inicial not in arvore_minima:
                arvore_minima[v_inicial] = (None, 0.0)

        return arvore_minima

    def relaxar(self, v, u, peso)->float:
        u_obj : Vertice = self.mapa_vertices[u]
        v_obj : Vertice = self.mapa_vertices[v]
        u_obj.rotulo = v_obj.rotulo + peso
        
    def djikstra(self, v_inicial : Hashable):
        """retorna os caminhos mínimos a partir de um vértice"""
        self.reseta_busca()
        
        self.min_heap : List [List[float, Hashable]] = [] # (rotulo, id)
        heapq.heapify(self.min_heap)
        
        for v in self.lista_adjacencia:           
            if v != v_inicial:
                heapq.heappush(self.min_heap, [float('inf'), v ])
            else: 
                heapq.heappush(self.min_heap, [0.0, v ])
                v_inicial_obj : Vertice = self.mapa_vertices[v_inicial]
                v_inicial_obj.rotulo = 0.0

        while self.min_heap:
            par_min = heapq.heappop(self.min_heap)
            min : Hashable = par_min[1]
              
            min_obj = self.mapa_vertices[min]
            if min_obj.esta_marcado():
                continue # já processado antes (duplicata do heap)
            min_obj.marcar()
            

            for v in self.get_adjacentes(min):
                v_obj = self.mapa_vertices[v]
                peso_aresta = self.lista_adjacencia[min][v]
                nova_dist = min_obj.rotulo + peso_aresta
                if not v_obj.esta_marcado() and nova_dist < v_obj.rotulo:     
                    v_obj.antecessor = min
                    self.relaxar(min, v, peso_aresta)
                    #self._atualiza_rotulo_heap(v, peso_aresta)
                    heapq.heappush(self.min_heap, [peso_aresta, v])
    
    def reconstruir_caminho(self, origem, destino):
        caminho = []
        atual = destino

        while atual is not None:
            atual = self.mapa_vertices[atual]
            caminho.append((atual.index, atual.rotulo))
            atual = atual.antecessor
            

        caminho.reverse()
        if caminho[0][0] != origem:
            return None  # não há caminho
        return caminho

    def bellman_ford(self, v_inicial):
        self.reseta_busca()
                
        for v in self.lista_adjacencia:           
            if v == v_inicial: 
                v_inicial_obj : Vertice = self.mapa_vertices[v_inicial]
                v_inicial_obj.rotulo = 0.0
                
        for i in range(0, self.get_numero_vertices()-1):
            for v1 in self.lista_adjacencia:
                for v2 in self.lista_adjacencia[v1]:
                    p = self.lista_adjacencia[v1][v2]
                    self.relaxar(v1, v2, p)

        for v1 in self.lista_adjacencia:
                for v2 in self.lista_adjacencia[v1]:
                    p = self.lista_adjacencia[v1][v2]
                    peso_v1 = self.mapa_vertices[v1].rotulo
                    peso_v2 = self.mapa_vertices[v2].rotulo
                    
                    if peso_v1 > peso_v2 + p:
                        return False
        return True

class GrafoComFluxo(GrafoPonderado):
    """
    extensão GrafoPonderado que permite cálculos de fluxo máximo
    usando Ford-Fulkerson e Capacity Scaling.
    """

    def __init__(self, direcionado=True):
        super().__init__(direcionado)
        self.capacidade: Dict[Tuple[Hashable, Hashable], float] = {}
        self.fluxo: Dict[Tuple[Hashable, Hashable], float] = {}

    def inserir_aresta(self, u, v, capacidade, insere_vertice=True):
        """Insere aresta com capacidade (e cria aresta reversa se necessário)."""
        super().inserir_vertice(u)
        super().inserir_vertice(v)
        self.lista_adjacencia.setdefault(u, {})[v] = capacidade
        self.capacidade[(u, v)] = capacidade
        self.fluxo[(u, v)] = 0
        # Aresta reversa (com capacidade 0, usada no grafo residual)
        if (v, u) not in self.capacidade:
            self.capacidade[(v, u)] = 0
            self.fluxo[(v, u)] = 0

    def grafo_residual(self):
        """Retorna o grafo residual Gf com base no fluxo atual."""
        Gf = GrafoComFluxo(direcionado=True)
        for (u, v), cap in self.capacidade.items():
            fluxo = self.fluxo.get((u, v), 0)
            res = cap - fluxo
            if res > 0:  # ainda há capacidade residual
                Gf.inserir_aresta(u, v, res)
        return Gf

    def caminho_aumentante(self, s, t):
        """Busca caminho de s até t no grafo residual."""
        pai = {s: None}
        fila = deque([s])

        while fila:
            u = fila.popleft()
            for v in self.lista_adjacencia[u]:
                if v not in pai and self.lista_adjacencia[u][v] > 0:
                    pai[v] = u
                    if v == t:
                        # reconstrói caminho
                        P = []
                        atual = t
                        while atual is not None:
                            ant = pai[atual]
                            if ant is not None:
                                P.append((ant, atual))
                            atual = ant
                        P.reverse()
                        return P
                    fila.append(v)
        return None  # nenhum caminho


    def gargalo(self, P):
        """Retorna o gargalo (capacidade mínima) do caminho P."""
        return min(self.lista_adjacencia[u][v] for (u, v) in P)

    def aumentar_fluxo(self, P):
        """Aumenta o fluxo ao longo de um caminho P."""
        r = self.gargalo(P)
        for (u, v) in P:
            # Aresta direta
            if (u, v) in self.fluxo:
                self.fluxo[(u, v)] += r
            # Aresta reversa
            else:
                self.fluxo[(v, u)] -= r
        return r

    def ford_fulkerson(self, s, t):
        """Algoritmo de Ford-Fulkerson (Fluxo Máximo)."""
        for (u, v) in self.capacidade:
            self.fluxo[(u, v)] = 0

        fluxo_max = 0
        while True:
            Gf = self.grafo_residual()
            P = Gf.caminho_aumentante(s, t)
            if not P:
                break  # nenhum caminho aumentante restante
            r = self.gargalo(P)
            self.aumentar_fluxo(P)
            fluxo_max += r

        return fluxo_max

    def capacity_scalling(self, s, t):
        """Algoritmo de Capacity Scaling para Fluxo Máximo."""
        for (u, v) in self.capacidade:
            self.fluxo[(u, v)] = 0

        # delta inicial = 2^floor(log2(max capacidade de saída de s))
        max_cap = max(self.lista_adjacencia[s].values(), default=0)
        delta = 1
        while delta * 2 <= max_cap:
            delta *= 2

        fluxo_max = 0
        while delta > 0:
            # Grafo residual delta-filtrado
            Gf = GrafoComFluxo(direcionado=True)
            for (u, v), cap in self.capacidade.items():
                fluxo = self.fluxo.get((u, v), 0)
                res = cap - fluxo
                if res >= delta:
                    Gf.inserir_aresta(u, v, res)

            # enquanto existir caminho aumentante delta-válido
            while True:
                P = Gf.caminho_aumentante(s, t)
                if not P:
                    break
                r = self.gargalo(P)
                self.aumentar_fluxo(P)
                fluxo_max += r
                # atualiza grafo residual delta-filtrado
                Gf = GrafoComFluxo(direcionado=True)
                for (u, v), cap in self.capacidade.items():
                    fluxo = self.fluxo.get((u, v), 0)
                    res = cap - fluxo
                    if res >= delta:
                        Gf.inserir_aresta(u, v, res)

            delta //= 2

        return fluxo_max
