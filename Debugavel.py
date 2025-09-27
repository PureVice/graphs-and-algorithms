
# %%
import Grafo as g_lib

# %% [markdown]
# Montagem de um grafo arbitrário
# 

# %%
g = g_lib.Grafo()
lista = [0, 1, 2, 3, 4, 5, 6]
g.inserir_lista_vertices(lista)



# %%
g

# %%
g.inserir_aresta(0,1)
g.inserir_aresta(1,3)
g.inserir_aresta(1,2)
g.inserir_aresta(2,3)
g.inserir_aresta(2,4)
g.inserir_aresta(4,6)
g.inserir_aresta(6,5)


# %%
num_arestas = g.get_numero_arestas()
num_vertices = g.get_numero_vertices()

print(f"arestas: {num_arestas} \nvertices: {num_vertices}")

# %%
g

# %%
#g.dfs()

g.dfs_iterativa(1)
# %%
g.imprimir_caminho(0, 1)


# %%
