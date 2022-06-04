
import matplotlib.pyplot as plt
import numpy as np
import math

recipiente = 17      # Peso do recipiente (gramas)
incremento = 50      # Quantidade de água adicionada a cada medida (gramas)
gravidade = 9.81     # Para cálculo gramas força
Conv = 0.00980665    # Converção gf para Newtons

X = []
Y = []
Yg = []
Yr = []
Desvio = []
Media = []

#------------------------------------ DADOS MEDIDOS NO SENSOR [SIEMENS] --------------------------------------------------||
X.append([6.32, 6.83, 6.58, 6.58, 7.14, 6.93, 6.93, 7.18, 8.44, 7.43, 7.41, 7.41, 7.18, 7.26, 7.55, 7.49]) #Medida (gramas agua): 0
X.append([30.61, 30.51, 30.61, 30.48, 30.55, 30.65, 30.92, 30.68, 30.72, 30.72, 30.78, 30.82, 32.53, 30.89, 30.96, 30.92]) #Medida (gramas agua): 50
X.append([41.04, 43.02, 40.96, 40.66, 41.13, 41.51, 41.25, 41.13, 41.72, 41.08, 41.72, 41.34, 41.85, 41.76, 42.02, 41.76]) #Medida (gramas agua): 100
X.append([52.90, 50.15, 49.85, 52.48, 53.05, 51.10, 50.50, 52.43, 53.00, 50.45, 51.05, 52.32, 52.79, 52.58, 51.35, 53.89]) #Medida (gramas agua): 150
X.append([58.18, 58.70, 58.47, 58.58, 58.41, 58.64, 58.58, 58.70, 58.70, 59.22, 58.87, 58.93, 59.10, 58.93, 58.64, 56.22]) #Medida (gramas agua): 200
X.append([63.41, 63.47, 63.16, 63.59, 62.98, 63.04, 63.22, 63.66, 63.29, 63.22, 63.16, 63.97, 63.97, 63.97, 63.97, 64.66]) #Medida (gramas agua): 250
X.append([71.79, 71.65, 71.86, 71.79, 72.07, 71.65, 72.22, 72.43, 72.36, 72.57, 72.50, 72.57, 72.57, 72.57, 72.43, 72.57]) #Medida (gramas agua): 300
X.append([79.37, 79.06, 78.51, 78.51, 79.68, 79.21, 78.51, 79.37, 79.68, 79.06, 78.98, 79.84, 79.53, 78.90, 79.61, 80.00]) #Medida (gramas agua): 350
X.append([84.24, 86.36, 84.57, 84.57, 84.74, 84.74, 84.82, 84.57, 84.57, 84.99, 84.99, 84.57, 84.99, 84.99, 84.99, 85.76]) #Medida (gramas agua): 400
X.append([90.53, 90.62, 90.16, 90.26, 90.53, 90.62, 90.62, 90.71, 90.89, 90.89, 90.71, 90.89, 90.99, 90.89, 91.35, 91.17]) #Medida (gramas agua): 450
X.append([96.10, 96.49, 96.59, 96.69, 91.63, 96.79, 96.49, 96.79, 96.49, 96.88, 97.38, 96.79, 96.20, 98.28, 96.79, 97.18]) #Medida (gramas agua): 500

#-------------------------------------------------------------------------------------------------------------------------||

fig, ax = plt.subplots(figsize=(10, 6))
fig, dv = plt.subplots(figsize=(10, 6))

#--------- Converção de gramas (água + recipeiente) para Newtons e Poltagem nos resultados ----------------------------------||
for i in range(len(X)):
       Y.append((np.ones(len(X[i]))*i*incremento+recipiente)*gravidade*Conv)  # Eixo da força [N] para cada amostra da medida i
       ax.plot(X[i],Y[i], color = 'blue', marker = 'o')                       # 16 amostras para a medida i
       Media.append(sum(X[i])/len(X[i]))                                      # Média das 16 amostras da medida i
       Yg.append(i*50+recipiente)                                             # Vetor de gramas plicadas com água + recipiente
       Yr.append((i*50+recipiente)*gravidade*Conv)                            # Eixo da força para medida i

#----- Interpolação entre a média simples de amostra e a força aplicada -------------------------------||
p = np.polyfit(Media,Yr,2)         # Cálculo da equação de segundo grau
print(p)
yresult = np.polyval(p, Media)     # Eixo da força [N] calculada com a média das amostras

#------------------------- Cálculo do desvio padrão após aplicar a equação nas amostras brutas-----------||
for i in range(len(X)):
       des_result = np.polyval(p, X[i])   # Força [N] calculada para cada amostra
       Desvio.append(math.sqrt(sum(pow(des_result-sum(des_result)/len(des_result),2))/len(des_result)))    # Cálculo do desvio para a medida i

#------------- Polotagem dos resultados ---------------------------------------------------||
dv.plot(Yr, Desvio, color = 'blue', marker = 'x') #

dv.set(title = "Desvio padrão de 16 amostras das 20 medidas",
       xlabel = "Força Aplicada [N]", 
       ylabel = "Desvio padrão [N]")

ax.set(title = "Força x condutância",
       xlabel = "Condutância no resistor variável Rp [S]", 
       ylabel = "Força Aplicada [N]")

ax.scatter(Media,yresult, color = 'red', marker = 'x', label = 'Média da "força"', s = 100)
ax.plot(Media,yresult, color = 'red', label = 'Regressão de segundo grau')

for i, txt in enumerate(Yg):
    ax.annotate(txt, xy = (Media[i], yresult[i]), xytext = (Media[i] - 4.5, yresult[i] + 5))
    dv.annotate(txt, (Yr[i]+1, Desvio[i]+3))

ax.grid()
ax.legend()
plt.yticks(Yr)

#print(X)

plt.show()
